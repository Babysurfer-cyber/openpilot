import math
import os

from opendbc import DBC_PATH
from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.values import DBC, HyundaiFlags, HyundaiExtFlags
from openpilot.common.params import Params
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.radar_tracks import MyTrack

SCC_TID = 0
RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32
RADAR_START_ADDR_CANFD1 = 0x210
RADAR_MSG_COUNT1 = 16
RADAR_START_ADDR_CANFD2 = 0x3A5 # Group 2, Group 1: 0x210 2개씩있어서 일단 보류.
RADAR_MSG_COUNT2 = 32

CORNER_OBJECT_235_START_ADDR = 0x235
CORNER_OBJECT_235_MSG_COUNT = 20
CORNER_OBJECT_235_TRACK_ID_OFFSET = 200
CORNER_OBJECT_235_DBC = 'hyundai_canfd_corner_radar_235_generated'
CORNER_OBJECT_235_STALE_FRAMES = 50
CORNER_OBJECT_180_START_ADDR = 0x180
CORNER_OBJECT_180_MSG_COUNT = 5
CORNER_OBJECT_180_SLOTS_PER_MSG = 2
CORNER_OBJECT_180_TRACK_ID_OFFSET = 240
CORNER_OBJECT_180_DBC = 'hyundai_canfd_corner_radar_180_generated'
CORNER_OBJECT_180_STALE_FRAMES = 50  # update() runs at the 100 Hz car interface cadence
CORNER_OBJECT_STABLE_TRACK_ID_START = 1000
CORNER_OBJECT_IDENTITY_STALE_CYCLES = 3
CORNER_OBJECT_IDENTITY_MAX_DREL_DELTA = 7.0
CORNER_OBJECT_IDENTITY_MAX_YREL_DELTA = 3.2
CORNER_OBJECT_HANDOFF_MAX_DREL_DELTA = 2.0
CORNER_OBJECT_HANDOFF_MAX_YREL_DELTA = 1.0
CORNER_OBJECT_HANDOFF_MAX_VREL_DELTA = 3.0


class CornerObjectTrackIdManager:
  def __init__(self):
    self.next_track_id = CORNER_OBJECT_STABLE_TRACK_ID_START
    self.source_cycles = {}
    self.track_states = {}

  def clear_source(self, source):
    self.track_states = {key: state for key, state in self.track_states.items() if key[0] != source}
    self.source_cycles.pop(source, None)

  def get_track_ids(self, source, candidates):
    cycle = self.source_cycles.get(source, 0) + 1
    self.source_cycles[source] = cycle
    previous = {
      track_id: state for (state_source, track_id), state in self.track_states.items()
      if state_source == source and cycle - state[5] <= CORNER_OBJECT_IDENTITY_STALE_CYCLES
    }
    used_track_ids = set()
    assignments = {}
    for slot_id, object_id, age, _, d_rel, y_rel, *_ in candidates:
      matches = []
      for track_id, state in previous.items():
        previous_slot, previous_object_id, previous_age, previous_d_rel, previous_y_rel, _ = state
        if track_id in used_track_ids or object_id != previous_object_id or age < previous_age:
          continue
        d_delta = abs(d_rel - previous_d_rel)
        y_delta = abs(y_rel - previous_y_rel)
        if d_delta <= CORNER_OBJECT_IDENTITY_MAX_DREL_DELTA and y_delta <= CORNER_OBJECT_IDENTITY_MAX_YREL_DELTA:
          matches.append((previous_slot != slot_id, d_delta + y_delta * 1.5, track_id))
      if matches:
        track_id = min(matches)[2]
      else:
        track_id = self.next_track_id
        self.next_track_id += 1
      assignments[slot_id] = track_id
      used_track_ids.add(track_id)
      self.track_states[(source, track_id)] = (slot_id, object_id, age, d_rel, y_rel, cycle)

    self.track_states = {
      key: state for key, state in self.track_states.items()
      if key[0] != source or cycle - state[5] <= CORNER_OBJECT_IDENTITY_STALE_CYCLES
    }
    return assignments


def deduplicate_corner_candidates(candidates):
  objects = []
  for candidate in candidates:
    _, object_id, age, quality, d_rel, y_rel, v_rel, *_ = candidate
    duplicate_index = None
    for index, previous in enumerate(objects):
      if (object_id == previous[1] and
          abs(d_rel - previous[4]) <= CORNER_OBJECT_HANDOFF_MAX_DREL_DELTA and
          abs(y_rel - previous[5]) <= CORNER_OBJECT_HANDOFF_MAX_YREL_DELTA and
          abs(v_rel - previous[6]) <= CORNER_OBJECT_HANDOFF_MAX_VREL_DELTA):
        duplicate_index = index
        break
    if duplicate_index is None:
      objects.append(candidate)
    elif (age, quality) > (objects[duplicate_index][2], objects[duplicate_index][3]):
      objects[duplicate_index] = candidate
  return objects


def corner_object_position_valid(d_rel, y_rel):
  normal_object = 0.2 < d_rel < 180.0
  clipped_side_object = 0.0 <= d_rel <= 0.2 and 1.4 <= abs(y_rel) <= 4.5
  return (normal_object or clipped_side_object) and abs(y_rel) < 40.0

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/

def get_radar_can_parser(CP, radar_tracks, msg_start_addr, msg_count):
  if not radar_tracks:
    return None
  #if Bus.radar not in DBC[CP.carFingerprint]:
  #  return None
  print("RadarInterface: RadarTracks...")

  if CP.flags & HyundaiFlags.CANFD:
    CAN = CanBus(CP)
    messages = [(f"RADAR_TRACK_{addr:x}", 20) for addr in range(msg_start_addr, msg_start_addr + msg_count)]
    return CANParser('hyundai_canfd_radar_generated', messages, CAN.ACAN)
  else:
    messages = [(f"RADAR_TRACK_{addr:x}", 20) for addr in range(msg_start_addr, msg_start_addr + msg_count)]
  #return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)
    return CANParser('hyundai_kia_mando_front_radar_generated', messages, 1)

def get_radar_can_parser_scc(CP):
  CAN = CanBus(CP)
  if CP.flags & HyundaiFlags.CANFD:
    messages = [("SCC_CONTROL", 50)]
    bus = CAN.ECAN
  else:
    messages = [("SCC11", 50)]
    bus = CAN.ECAN

  print("$$$$$$$$ ECAN = ", CAN.ECAN)    
  bus = CAN.CAM if CP.flags & HyundaiFlags.CAMERA_SCC else bus
  return CANParser(DBC[CP.carFingerprint][Bus.pt], messages, bus)


def get_corner_object_180_can_parser(CP, enabled):
  if not enabled or not (CP.flags & HyundaiFlags.CANFD):
    return None
  if not os.path.exists(os.path.join(DBC_PATH, f"{CORNER_OBJECT_180_DBC}.dbc")):
    print(f"RadarInterface: missing {CORNER_OBJECT_180_DBC}.dbc, 0x180 corner radar disabled")
    return None
  messages = [(f"CORNER_RADAR_180_OBJECTS_{addr:x}", 33)
              for addr in range(CORNER_OBJECT_180_START_ADDR, CORNER_OBJECT_180_START_ADDR + CORNER_OBJECT_180_MSG_COUNT)]
  return CANParser(CORNER_OBJECT_180_DBC, messages, CanBus(CP).ACAN)


def get_corner_object_can_parser(CP, enabled):
  if not enabled or not (CP.flags & HyundaiFlags.CANFD):
    return None
  if not os.path.exists(os.path.join(DBC_PATH, f"{CORNER_OBJECT_235_DBC}.dbc")):
    print(f"RadarInterface: missing {CORNER_OBJECT_235_DBC}.dbc, 0x235 corner radar disabled")
    return None
  messages = [(f"CORNER_RADAR_235_OBJECTS_{addr:x}", 33)
              for addr in range(CORNER_OBJECT_235_START_ADDR, CORNER_OBJECT_235_START_ADDR + CORNER_OBJECT_235_MSG_COUNT)]
  return CANParser(CORNER_OBJECT_235_DBC, messages, CanBus(CP).ACAN)


get_corner_object_235_can_parser = get_corner_object_can_parser

class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    
    self.canfd = True if CP.flags & HyundaiFlags.CANFD else False
    self.radar_group1 = False
    if self.canfd:
      if CP.extFlags & HyundaiExtFlags.RADAR_GROUP1.value:
        self.radar_start_addr = RADAR_START_ADDR_CANFD1
        self.radar_msg_count = RADAR_MSG_COUNT1
        self.radar_group1 = True
      else:
        self.radar_start_addr = RADAR_START_ADDR_CANFD2
        self.radar_msg_count = RADAR_MSG_COUNT2
    else:
      self.radar_start_addr = RADAR_START_ADDR
      self.radar_msg_count = RADAR_MSG_COUNT
      
    self.params = Params()
    self.radar_tracks = self.params.get_int("EnableRadarTracks") >= 1
    self.corner_object_180_tracks = (bool(CP.extFlags & HyundaiExtFlags.CORNER_RADAR_OBJECTS_180.value) and
                                     self.params.get_int("EnableCornerRadar") > 0)
    self.corner_object_tracks = (bool(CP.extFlags & HyundaiExtFlags.CORNER_RADAR_OBJECTS_235.value) and
                                 self.params.get_int("EnableCornerRadar") > 0)
    self.rcp_corner_objects = get_corner_object_can_parser(CP, self.corner_object_tracks)
    self.rcp_corner_objects_180 = get_corner_object_180_can_parser(CP, self.corner_object_180_tracks)
    self.corner_only = CP.radarUnavailable and (self.rcp_corner_objects is not None or self.rcp_corner_objects_180 is not None)
    self.corner_object_track_ids = CornerObjectTrackIdManager()
    self.corner_object_235_candidates = {}
    self.corner_object_235_last_seen = {}
    self.corner_object_235_point_ids = set()
    self.corner_object_180_candidates = {}
    self.corner_object_180_last_seen = {}
    self.corner_object_180_point_ids = set()
    self.updated_tracks = set()
    self.updated_scc = set()
    self.rcp_tracks = get_radar_can_parser(CP, self.radar_tracks and not self.corner_only, self.radar_start_addr, self.radar_msg_count)
    self.rcp_scc = get_radar_can_parser_scc(CP)
    self.trigger_msg_scc = 416 if self.canfd else 0x420

    self.trigger_msg_tracks = self.radar_start_addr + self.radar_msg_count - 1
    self.track_id = 0

    self.radar_off_can = CP.radarUnavailable and not self.corner_only

    self.vRel_last = 0
    self.dRel_last = 0

    # Initialize pts
    total_tracks = self.radar_msg_count * ( 2 if self.radar_group1 else 1)
    for track_id in range(total_tracks):
      t_id = track_id + 32
      self.pts[t_id] = structs.RadarData.RadarPoint()
      self.pts[t_id].measured = False
      self.pts[t_id].trackId = t_id

    self.pts[SCC_TID] = structs.RadarData.RadarPoint()
    self.pts[SCC_TID].trackId = SCC_TID
    self.pts[SCC_TID].radarSource = "scc"

    self.frame = 0


  def update(self, can_strings):
    self.frame += 1
    if self.radar_off_can or (self.rcp_tracks is None and self.rcp_scc is None and self.rcp_corner_objects is None and self.rcp_corner_objects_180 is None):
      return super().update(None)

    if self.rcp_corner_objects is not None:
      updated_corner = self.rcp_corner_objects.update(can_strings)
      self._update_corner_objects(updated_corner)
    if self.rcp_corner_objects_180 is not None:
      updated_corner = self.rcp_corner_objects_180.update(can_strings)
      self._update_corner_objects_180(updated_corner)

    if self.rcp_scc is not None:
      vls_s = self.rcp_scc.update(can_strings)
      self.updated_scc.update(vls_s)
      if (not self.radar_tracks or self.corner_only) and self.frame % 5 == 0:
        if not self.corner_only or self.rcp_scc.can_valid:
          self._update_scc(self.updated_scc)
        else:
          self._clear_point(SCC_TID)
        self.updated_scc.clear()
        ret = structs.RadarData()
        if ((not self.corner_only and not self.rcp_scc.can_valid) or
            (self.rcp_corner_objects is not None and not self.rcp_corner_objects.can_valid) or
            (self.rcp_corner_objects_180 is not None and not self.rcp_corner_objects_180.can_valid)):
          ret.errors.canError = True
        ret.points = list(self.pts.values())
        return ret
    if self.radar_tracks and self.rcp_tracks is not None:
      vls_t = self.rcp_tracks.update(can_strings)
      self.updated_tracks.update(vls_t)
      if self.trigger_msg_tracks in self.updated_tracks:
        self._update(self.updated_tracks)
        self._update_scc(self.updated_scc)
        self.updated_scc.clear()
        self.updated_tracks.clear()
        ret = structs.RadarData()
        if (not self.rcp_tracks.can_valid or
            (self.rcp_corner_objects is not None and not self.rcp_corner_objects.can_valid) or
            (self.rcp_corner_objects_180 is not None and not self.rcp_corner_objects_180.can_valid)):
          ret.errors.canError = True
        ret.points = list(self.pts.values())
        return ret

    return None      

  def update_carrot(self, v_ego, a_ego, rcv_time, can_packets):
    if self.rcp_corner_objects is None and self.rcp_corner_objects_180 is None:
      return super().update_carrot(v_ego, a_ego, rcv_time, can_packets)
    # Use the upstream identified-track observer locally to Hyundai. The old
    # generic filter reset a stable track's history during real braking, which
    # destroys the stationary/cut-out evidence consumed by radar_motion.
    self.v_ego_hist.append(v_ego)
    self.v_ego = self.v_ego_hist[0]
    self.a_ego_hist.append(a_ego)
    self.a_ego = self.a_ego_hist[0]
    ret = self.update(can_packets)
    if ret is not None:
      if not self.init_done:
        self.estimate_dt(rcv_time)
        return None
      new_tracks = {}
      for point in self.pts.values():
        track_id = point.trackId
        track = self.tracks.get(track_id)
        if track is None:
          track = MyTrack(track_id, point, self.dt)
        track.update(point, self.a_ego)
        track.write_acceleration(point)
        point.yRel = float(track.yRel)
        point.yvRel = float(track.yvRel)
        new_tracks[track_id] = track
      self.tracks = new_tracks
      # update() has already copied the points. Publish this observation's
      # dynamics with its current range and retain its native sensor metadata.
      for point in ret.points:
        track = self.tracks.get(point.trackId)
        if track is not None:
          track.write_acceleration(point)
          point.yRel = float(track.yRel)
          point.yvRel = float(track.yvRel)
    return ret

  def _update_corner_objects(self, updated_messages):
    if self.rcp_corner_objects is None:
      return

    changed = False
    for addr in range(CORNER_OBJECT_235_START_ADDR, CORNER_OBJECT_235_START_ADDR + CORNER_OBJECT_235_MSG_COUNT):
      if addr not in updated_messages:
        continue
      changed = True
      self.corner_object_235_last_seen[addr] = self.frame
      msg = self.rcp_corner_objects.vl[f"CORNER_RADAR_235_OBJECTS_{addr:x}"]
      d_rel = msg["OBJ_REL_POS_X"]
      y_rel = msg["OBJ_REL_POS_Y"]
      v_rel = msg["OBJ_REL_VEL_X"]
      quality = int(msg["OBJ_QUAL_LEVEL"])
      candidates = []
      if quality > 0 and corner_object_position_valid(d_rel, y_rel) and v_rel > -99.0:
        slot_id = CORNER_OBJECT_235_TRACK_ID_OFFSET + addr - CORNER_OBJECT_235_START_ADDR
        candidates.append((slot_id, int(msg["OBJ_OBJECT_ID"]), int(msg["OBJ_AGE"]), quality,
                           d_rel, y_rel, v_rel, msg["OBJ_REL_VEL_Y"], msg["OBJ_REL_ACCEL_X"]))
      self.corner_object_235_candidates[addr] = candidates

    for addr, last_seen in list(self.corner_object_235_last_seen.items()):
      if self.frame - last_seen > CORNER_OBJECT_235_STALE_FRAMES:
        changed = True
        self.corner_object_235_last_seen.pop(addr)
        self.corner_object_235_candidates.pop(addr, None)

    if changed:
      candidates = [candidate for message in self.corner_object_235_candidates.values() for candidate in message]
      self._apply_corner_objects("corner235", candidates)

  _update_corner_objects_235 = _update_corner_objects

  def _update_corner_objects_180(self, updated_messages):
    if self.rcp_corner_objects_180 is None:
      return

    changed = False
    for addr in range(CORNER_OBJECT_180_START_ADDR, CORNER_OBJECT_180_START_ADDR + CORNER_OBJECT_180_MSG_COUNT):
      if addr not in updated_messages:
        continue
      changed = True
      self.corner_object_180_last_seen[addr] = self.frame
      msg = self.rcp_corner_objects_180.vl[f"CORNER_RADAR_180_OBJECTS_{addr:x}"]
      candidates = []
      for slot_index in range(CORNER_OBJECT_180_SLOTS_PER_MSG):
        prefix = f"SLOT{slot_index + 1}_"
        d_rel = msg[f"{prefix}REL_POS_X"]
        y_rel = msg[f"{prefix}REL_POS_Y"]
        v_rel = msg[f"{prefix}REL_VEL_X"]
        quality = int(msg[f"{prefix}QUAL_LEVEL"])
        if quality > 0 and corner_object_position_valid(d_rel, y_rel) and v_rel > -99.0:
          slot_id = CORNER_OBJECT_180_TRACK_ID_OFFSET + (addr - CORNER_OBJECT_180_START_ADDR) * CORNER_OBJECT_180_SLOTS_PER_MSG + slot_index
          candidates.append((slot_id, int(msg[f"{prefix}OBJECT_ID"]), int(msg[f"{prefix}AGE"]), quality,
                             d_rel, y_rel, v_rel, msg[f"{prefix}REL_VEL_Y"], msg[f"{prefix}REL_ACCEL_X"]))
      self.corner_object_180_candidates[addr] = candidates

    # Expire each CAN message independently. Another slot's traffic must never
    # refresh a missing message's cached objects.
    for addr, last_seen in list(self.corner_object_180_last_seen.items()):
      if self.frame - last_seen > CORNER_OBJECT_180_STALE_FRAMES:
        changed = True
        self.corner_object_180_last_seen.pop(addr)
        self.corner_object_180_candidates.pop(addr, None)

    if changed:
      candidates = [candidate for message in self.corner_object_180_candidates.values() for candidate in message]
      self._apply_corner_objects("corner180", candidates)

  def _apply_corner_objects(self, source, candidates):
    objects = deduplicate_corner_candidates(candidates)
    track_ids = self.corner_object_track_ids.get_track_ids(source, objects)
    if not objects:
      self.corner_object_track_ids.clear_source(source)
    # Keep one point per stable identity. Unmeasured aliases in former CAN slots
    # would reset MyTrack's filter after an object moves to a new slot.
    point_ids_name = "corner_object_235_point_ids" if source == "corner235" else "corner_object_180_point_ids"
    point_ids = getattr(self, point_ids_name)
    for track_id in point_ids:
      self.pts.pop(track_id, None)
    point_ids.clear()
    for slot_id, _, _, _, d_rel, y_rel, v_rel, yv_rel, a_rel in objects:
      track_id = track_ids[slot_id]
      point = structs.RadarData.RadarPoint()
      point.trackId = track_id
      point.radarSource = source
      point.measured = True
      point.dRel = d_rel
      point.yRel = y_rel
      point.vRel = v_rel
      point.vLead = v_rel + self.v_ego
      point.yvRel = yv_rel
      point.aRel = a_rel
      self.pts[track_id] = point
      point_ids.add(track_id)

  def _clear_point(self, track_id):
    point = self.pts[track_id]
    point.measured = False
    point.dRel = 0.0
    point.yRel = 0.0
    point.vRel = 0.0
    point.vLead = self.v_ego
    point.aRel = float('nan')
    point.yvRel = 0.0
    point.aLead = 0.0
    point.jLead = 0.0
    point.trackState = 0

  def _update(self, updated_messages):

    t_id = 32
    for addr in range(self.radar_start_addr, self.radar_start_addr + self.radar_msg_count):

      msg = self.rcp_tracks.vl[f"RADAR_TRACK_{addr:x}"]

      if self.radar_group1:
        valid = msg['VALID_CNT1'] > 10
        track_state = 0  # VALID_CNT is an age counter, not an object state.
      elif self.canfd:
        valid = msg['VALID_CNT'] > 10
        track_state = int(msg['VALID'])
      else:
        valid = msg['STATE'] in (3, 4)
        track_state = int(msg['STATE'])

      self.pts[t_id].measured = bool(valid)
      self.pts[t_id].trackState = track_state
      if not valid:
        self.pts[t_id].dRel = 0
        self.pts[t_id].yRel = 0
        self.pts[t_id].vRel = 0
        self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
        self.pts[t_id].aRel = float('nan')
        self.pts[t_id].yvRel = 0
      elif self.radar_group1:
        self.pts[t_id].dRel = msg['LONG_DIST1']
        self.pts[t_id].yRel = msg['LAT_DIST1']
        self.pts[t_id].vRel = msg['REL_SPEED1']
        self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
        self.pts[t_id].aRel = msg['REL_ACCEL1']
        self.pts[t_id].yvRel = msg['LAT_SPEED1']
      elif self.canfd:
        self.pts[t_id].dRel = msg['LONG_DIST']
        self.pts[t_id].yRel = msg['LAT_DIST']
        self.pts[t_id].vRel = msg['REL_SPEED']
        self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
        self.pts[t_id].aRel = msg['REL_ACCEL']
        self.pts[t_id].yvRel = msg['LAT_SPEED']
      else:
        azimuth = math.radians(msg['AZIMUTH'])
        self.pts[t_id].dRel = math.cos(azimuth) * msg['LONG_DIST']
        self.pts[t_id].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
        self.pts[t_id].vRel = msg['REL_SPEED']
        self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
        self.pts[t_id].aRel = msg['REL_ACCEL']
        self.pts[t_id].yvRel = 0.0

      t_id += 1
    # radar group1은 하나의 msg에 2개의 레이더가 들어있음.
    if self.radar_group1:
      for addr in range(self.radar_start_addr, self.radar_start_addr + self.radar_msg_count):
        msg = self.rcp_tracks.vl[f"RADAR_TRACK_{addr:x}"]

        valid = msg['VALID_CNT2'] > 10
        self.pts[t_id].measured = bool(valid)
        self.pts[t_id].trackState = 0
        if not valid:
          self.pts[t_id].dRel = 0
          self.pts[t_id].yRel = 0
          self.pts[t_id].vRel = 0
          self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
          self.pts[t_id].aRel = float('nan')
          self.pts[t_id].yvRel = 0
        else:
          self.pts[t_id].dRel = msg['LONG_DIST2']
          self.pts[t_id].yRel = msg['LAT_DIST2']
          self.pts[t_id].vRel = msg['REL_SPEED2']
          self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
          self.pts[t_id].aRel = msg['REL_ACCEL2']
          self.pts[t_id].yvRel = msg['LAT_SPEED2']

        t_id += 1

  def _update_scc(self, updated_messages):
    cpt = self.rcp_scc.vl
    t_id = SCC_TID
    if self.canfd:
      dRel = cpt["SCC_CONTROL"]['ACC_ObjDist']
      vRel = cpt["SCC_CONTROL"]['ACC_ObjRelSpd']
      new_pts = abs(dRel - self.dRel_last) > 3 or abs(vRel - self.vRel_last) > 1
      vLead = vRel + self.v_ego
      valid = 0 < dRel < 150 and not new_pts #cpt["SCC_CONTROL"]['OBJ_STATUS'] and dRel < 150
      self.pts[t_id].measured = bool(valid)
      if not valid:
        self.pts[t_id].dRel = 0
        self.pts[t_id].yRel = 0
        self.pts[t_id].vRel = 0
        self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
        self.pts[t_id].aRel = float('nan')
        self.pts[t_id].yvRel = 0
      else:
        self.pts[t_id].dRel = dRel
        self.pts[t_id].yRel = 0
        self.pts[t_id].vRel = vRel
        self.pts[t_id].vLead = vLead
        self.pts[t_id].aRel = float('nan')
        self.pts[t_id].yvRel = 0 #float('nan')
    else:
      dRel = cpt["SCC11"]['ACC_ObjDist']
      vRel = cpt["SCC11"]['ACC_ObjRelSpd']
      new_pts = abs(dRel - self.dRel_last) > 3 or abs(vRel - self.vRel_last) > 1
      vLead = vRel + self.v_ego
      valid = cpt["SCC11"]['ACC_ObjStatus'] and dRel < 150 and not new_pts
      self.pts[t_id].measured = bool(valid)
      if not valid:
        self.pts[t_id].dRel = 0
        self.pts[t_id].yRel = 0
        self.pts[t_id].vRel = 0
        self.pts[t_id].vLead = self.pts[t_id].vRel + self.v_ego
        self.pts[t_id].aRel = float('nan')
        self.pts[t_id].yvRel = 0
      else:
        self.pts[t_id].dRel = dRel
        self.pts[t_id].yRel = -cpt["SCC11"]['ACC_ObjLatPos']  # in car frame's y axis, left is negative
        self.pts[t_id].vRel = vRel
        self.pts[t_id].vLead = vLead
        self.pts[t_id].aRel = float('nan')
        self.pts[t_id].yvRel = 0 #float('nan')

    self.dRel_last = dRel
    self.vRel_last = vRel
