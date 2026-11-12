import math
import os
from opendbc import DBC_PATH

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.values import DBC, HyundaiFlags, HyundaiExtFlags
from openpilot.common.params import Params
from opendbc.car.hyundai.hyundaicanfd import CanBus
from openpilot.common.filter_simple import MyMovingAverage

# --- Corner Radar Constants ---
CORNER_OBJECT_235_START_ADDR = 0x235
CORNER_OBJECT_235_MSG_COUNT = 20
CORNER_OBJECT_235_TRACK_ID_OFFSET = 200
CORNER_OBJECT_235_DBC = 'hyundai_canfd_corner_radar_235_generated'

CORNER_OBJECT_180_START_ADDR = 0x180
CORNER_OBJECT_180_MSG_COUNT = 5
CORNER_OBJECT_180_TRACK_ID_OFFSET = 240
CORNER_OBJECT_180_DBC = 'hyundai_canfd_corner_radar_180_generated'

CORNER_OBJECT_STABLE_TRACK_ID_START = 1000
CORNER_OBJECT_IDENTITY_STALE_CYCLES = 3
CORNER_OBJECT_IDENTITY_MAX_DREL_DELTA = 7.0
CORNER_OBJECT_IDENTITY_MAX_YREL_DELTA = 3.2
CORNER_OBJECT_HANDOFF_MAX_DREL_DELTA = 2.0
CORNER_OBJECT_HANDOFF_MAX_YREL_DELTA = 1.0
CORNER_OBJECT_HANDOFF_MAX_VREL_DELTA = 3.0
CORNER_SIDE_OBJECT_MAX_DREL = 0.2
CORNER_SIDE_OBJECT_MIN_ABS_YREL = 1.4
CORNER_SIDE_OBJECT_MAX_ABS_YREL = 4.5

class CornerObjectTrackIdManager:
  def __init__(self):
    self.next_track_id = CORNER_OBJECT_STABLE_TRACK_ID_START
    self.source_cycles, self.track_states = {}, {}
  def clear_source(self, source: str):
    self.track_states = {key: value for key, value in self.track_states.items() if key[0] != source}
    self.source_cycles.pop(source, None)
  def get_track_ids(self, source: str, candidates):
    cycle = self.source_cycles.get(source, 0) + 1
    self.source_cycles[source] = cycle
    previous = {track_id: state for (state_source, track_id), state in self.track_states.items() if state_source == source and cycle - state[5] <= CORNER_OBJECT_IDENTITY_STALE_CYCLES}
    used_track_ids, assignments = set(), {}
    for candidate in candidates:
      slot_id, object_id, age, _, d_rel, y_rel, *_ = candidate
      matches = []
      for track_id, state in previous.items():
        prev_slot, prev_object_id, prev_age, prev_d_rel, prev_y_rel, _ = state
        if track_id in used_track_ids or object_id != prev_object_id or age < prev_age: continue
        d_delta, y_delta = abs(d_rel - prev_d_rel), abs(y_rel - prev_y_rel)
        if d_delta > CORNER_OBJECT_IDENTITY_MAX_DREL_DELTA or y_delta > CORNER_OBJECT_IDENTITY_MAX_YREL_DELTA: continue
        matches.append((prev_slot != slot_id, d_delta + y_delta * 1.5, track_id))
      track_id = min(matches)[2] if matches else self.next_track_id
      if not matches: self.next_track_id += 1
      assignments[slot_id] = track_id
      used_track_ids.add(track_id)
      self.track_states[(source, track_id)] = (slot_id, object_id, age, d_rel, y_rel, cycle)
    self.track_states = {key: state for key, state in self.track_states.items() if key[0] != source or cycle - state[5] <= CORNER_OBJECT_IDENTITY_STALE_CYCLES}
    return assignments

def deduplicate_corner_candidates(candidates):
  objects = []
  for candidate in candidates:
    _, object_id, age, quality, d_rel, y_rel, v_rel, *_ = candidate
    dup_idx = next((i for i, prev in enumerate(objects) if object_id == prev[1] and abs(d_rel - prev[4]) <= CORNER_OBJECT_HANDOFF_MAX_DREL_DELTA and abs(y_rel - prev[5]) <= CORNER_OBJECT_HANDOFF_MAX_YREL_DELTA and abs(v_rel - prev[6]) <= CORNER_OBJECT_HANDOFF_MAX_VREL_DELTA), None)
    if dup_idx is None: objects.append(candidate)
    elif (age, quality) > (objects[dup_idx][2], objects[dup_idx][3]): objects[dup_idx] = candidate
  return objects

def corner_object_position_valid(d_rel: float, y_rel: float) -> bool:
  normal_object = 0.2 < d_rel < 180.0
  clipped_side_object = (0.0 <= d_rel <= CORNER_SIDE_OBJECT_MAX_DREL and CORNER_SIDE_OBJECT_MIN_ABS_YREL <= abs(y_rel) <= CORNER_SIDE_OBJECT_MAX_ABS_YREL)
  return (normal_object or clipped_side_object) and abs(y_rel) < 40.0

def get_corner_object_can_parsers(CP, enabled):
  if not enabled or not (CP.flags & HyundaiFlags.CANFD): return None, None
  
  CAN = CanBus(CP) # 👈 추가: CanBus 객체 생성
  
  parser_235, parser_180 = None, None
  if os.path.exists(os.path.join(DBC_PATH, f"{CORNER_OBJECT_235_DBC}.dbc")):
    msgs_235 = [(f"CORNER_RADAR_235_OBJECTS_{addr:x}", 33) for addr in range(CORNER_OBJECT_235_START_ADDR, CORNER_OBJECT_235_START_ADDR + CORNER_OBJECT_235_MSG_COUNT)]
    parser_235 = CANParser(CORNER_OBJECT_235_DBC, msgs_235, CAN.ACAN) # 👈 0을 CAN.ACAN으로 수정
    
  if os.path.exists(os.path.join(DBC_PATH, f"{CORNER_OBJECT_180_DBC}.dbc")):
    msgs_180 = [(f"CORNER_RADAR_180_OBJECTS_{addr:x}", 33) for addr in range(CORNER_OBJECT_180_START_ADDR, CORNER_OBJECT_180_START_ADDR + CORNER_OBJECT_180_MSG_COUNT)]
    parser_180 = CANParser(CORNER_OBJECT_180_DBC, msgs_180, CAN.ACAN) # 👈 0을 CAN.ACAN으로 수정
    
  return parser_235, parser_180

# --- Existing Constants ---
SCC_TID = 0
RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32
RADAR_START_ADDR_CANFD1 = 0x210
RADAR_MSG_COUNT1 = 16
RADAR_START_ADDR_CANFD2 = 0x3A5 # Group 2, Group 1: 0x210 2개씩있어서 일단 보류.
RADAR_MSG_COUNT2 = 32

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

class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    
    self.v_ego = 0.0 # 👈 추가: 크래시 방지용 초기값
    
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
    self.updated_tracks = set()
    self.updated_scc = set()
    self.rcp_tracks = get_radar_can_parser(CP, self.radar_tracks, self.radar_start_addr, self.radar_msg_count)
    self.rcp_scc = get_radar_can_parser_scc(CP)
    self.trigger_msg_scc = 416 if self.canfd else 0x420

    # (기존 코드)
    self.trigger_msg_tracks = self.radar_start_addr + self.radar_msg_count - 1
    self.track_id = 0
    # 👇 여기에 아래 두 줄 추가
    self.trigger_msg_corner_235 = CORNER_OBJECT_235_START_ADDR + CORNER_OBJECT_235_MSG_COUNT - 1
    self.trigger_msg_corner_180 = CORNER_OBJECT_180_START_ADDR + CORNER_OBJECT_180_MSG_COUNT - 1

    self.radar_off_can = CP.radarUnavailable

    self.vRel_last = 0
    self.dRel_last = 0

    # --- Corner Radar Initialization ---
    self.corner_object_tracks = self.params.get_int("EnableCornerRadar") > 0
    self.rcp_corner_235, self.rcp_corner_180 = get_corner_object_can_parsers(CP, self.corner_object_tracks)
    self.corner_object_track_ids = CornerObjectTrackIdManager()
    self.updated_corner_objects = set()
    self.corner_object_missed_updates = 0

    # Initialize pts
    total_tracks = self.radar_msg_count * ( 2 if self.radar_group1 else 1)
    for track_id in range(total_tracks):
      t_id = track_id + 32
      self.pts[t_id] = structs.RadarData.RadarPoint()
      self.pts[t_id].measured = False
      self.pts[t_id].trackId = t_id

    self.pts[SCC_TID] = structs.RadarData.RadarPoint()
    self.pts[SCC_TID].trackId = SCC_TID

    # Initialize pts for corner radar 0x235
    if self.rcp_corner_235 is not None:
      for slot in range(CORNER_OBJECT_235_MSG_COUNT):
        t_id = CORNER_OBJECT_235_TRACK_ID_OFFSET + slot
        self.pts[t_id] = structs.RadarData.RadarPoint()
        self.pts[t_id].measured = False
        self.pts[t_id].trackId = t_id

    # Initialize pts for corner radar 0x180
    if self.rcp_corner_180 is not None:
      for slot in range(CORNER_OBJECT_180_MSG_COUNT * 2):
        t_id = CORNER_OBJECT_180_TRACK_ID_OFFSET + slot
        self.pts[t_id] = structs.RadarData.RadarPoint()
        self.pts[t_id].measured = False
        self.pts[t_id].trackId = t_id

    self.frame = 0


  def update(self, can_strings):
    self.frame += 1
    if self.radar_off_can or (self.rcp_tracks is None and self.rcp_scc is None and self.rcp_corner_235 is None and self.rcp_corner_180 is None):
      return super().update(None)

    corner_235_ready = False
    corner_180_ready = False

    # --- Corner Radar CAN Update ---
    if self.rcp_corner_235 is not None:
      vls_235 = self.rcp_corner_235.update(can_strings)
      self.updated_corner_objects.update(vls_235)
      # 마지막 메시지가 수신되었는지 확인
      corner_235_ready = self.trigger_msg_corner_235 in self.updated_corner_objects

    if self.rcp_corner_180 is not None:
      vls_180 = self.rcp_corner_180.update(can_strings)
      # 180도 트리거 로직을 위한 set이 별도로 필요할 수 있지만 여기선 편의상 같이 담습니다
      self.updated_corner_objects.update(vls_180)
      corner_180_ready = self.trigger_msg_corner_180 in self.updated_corner_objects

    # --- Corner Radar 데이터 가공 (트리거 도달 시에만) ---
    if corner_235_ready or corner_180_ready:
      self._update_corner_objects(self.updated_corner_objects)
      self.corner_object_missed_updates = 0
      self.updated_corner_objects.clear()
    else:
      self.corner_object_missed_updates += 1
      if self.corner_object_missed_updates > 10: 
        self._clear_corner_objects()

    # --- Original Update Logic ---
    if self.rcp_scc is not None:
      vls_s = self.rcp_scc.update(can_strings)
      self.updated_scc.update(vls_s)
      
      if not self.radar_tracks and self.frame % 5 == 0:
        self._update_scc(self.updated_scc)
        self.updated_scc.clear()
        ret = structs.RadarData()
        if not self.rcp_scc.can_valid:
          ret.errors.canError = True
        # 👇 리턴 수정 (measured가 True인 것만)
        ret.points = [pt for pt in self.pts.values() if pt.measured] 
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
        # 코너 레이더 CAN Valid 에러 처리 추가
        can_error = not self.rcp_tracks.can_valid
        if self.rcp_corner_235 is not None and not self.rcp_corner_235.can_valid: can_error = True
        if self.rcp_corner_180 is not None and not self.rcp_corner_180.can_valid: can_error = True
        ret.errors.canError = can_error
        
        # 👇 리턴 수정 (measured가 True인 것만 반환)
        ret.points = [pt for pt in self.pts.values() if pt.measured]
        return ret

    return None   

  # --- Corner Radar Processing Methods ---
  def _update_corner_objects(self, updated_messages):
    if not updated_messages:
      self._clear_corner_objects()
      return
      
    candidates_235 = []
    if self.rcp_corner_235 is not None:
      for slot, addr in enumerate(range(CORNER_OBJECT_235_START_ADDR, CORNER_OBJECT_235_START_ADDR + CORNER_OBJECT_235_MSG_COUNT)):
        t_id = CORNER_OBJECT_235_TRACK_ID_OFFSET + slot
        msg = self.rcp_corner_235.vl[f"CORNER_RADAR_235_OBJECTS_{addr:x}"]
        d_rel, y_rel, v_rel, yv_rel, a_rel = msg["OBJ_REL_POS_X"], msg["OBJ_REL_POS_Y"], msg["OBJ_REL_VEL_X"], msg["OBJ_REL_VEL_Y"], msg["OBJ_REL_ACCEL_X"]
        if msg["OBJ_QUAL_LEVEL"] > 0 and corner_object_position_valid(d_rel, y_rel) and v_rel > -99.0:
          candidates_235.append((t_id, int(msg["OBJ_OBJECT_ID"]), int(msg["OBJ_AGE"]), int(msg["OBJ_QUAL_LEVEL"]), d_rel, y_rel, v_rel, yv_rel, a_rel))
          
    candidates_180 = []
    if self.rcp_corner_180 is not None:
      for i, addr in enumerate(range(CORNER_OBJECT_180_START_ADDR, CORNER_OBJECT_180_START_ADDR + CORNER_OBJECT_180_MSG_COUNT)):
        msg = self.rcp_corner_180.vl[f"CORNER_RADAR_180_OBJECTS_{addr:x}"]
        for obj_idx in (1, 2):
          t_id = CORNER_OBJECT_180_TRACK_ID_OFFSET + (i * 2) + (obj_idx - 1)
          qual = msg[f"OBJ{obj_idx}_QUAL_LEVEL"]
          d_rel, y_rel, v_rel, yv_rel, a_rel = msg[f"OBJ{obj_idx}_REL_POS_X"], msg[f"OBJ{obj_idx}_REL_POS_Y"], msg[f"OBJ{obj_idx}_REL_VEL_X"], msg[f"OBJ{obj_idx}_REL_VEL_Y"], msg[f"OBJ{obj_idx}_REL_ACCEL_X"]
          if qual > 0 and corner_object_position_valid(d_rel, y_rel) and v_rel > -99.0:
            candidates_180.append((t_id, int(msg[f"OBJ{obj_idx}_OBJECT_ID"]), int(msg[f"OBJ{obj_idx}_AGE"]), int(qual), d_rel, y_rel, v_rel, yv_rel, a_rel))

    if candidates_235 or self.rcp_corner_235 is not None:
      for t_id in range(CORNER_OBJECT_235_TRACK_ID_OFFSET, CORNER_OBJECT_235_TRACK_ID_OFFSET + CORNER_OBJECT_235_MSG_COUNT):
        self.pts[t_id].measured = False
      objs_235 = deduplicate_corner_candidates(candidates_235)
      ids_235 = self.corner_object_track_ids.get_track_ids("corner235", objs_235)
      for t_id, _, _, _, d_rel, y_rel, v_rel, yv_rel, a_rel in objs_235:
        self._set_corner_pt(t_id, ids_235[t_id], d_rel, y_rel, v_rel, a_rel, yv_rel)

    if candidates_180 or self.rcp_corner_180 is not None:
      for t_id in range(CORNER_OBJECT_180_TRACK_ID_OFFSET, CORNER_OBJECT_180_TRACK_ID_OFFSET + CORNER_OBJECT_180_MSG_COUNT * 2):
        self.pts[t_id].measured = False
      objs_180 = deduplicate_corner_candidates(candidates_180)
      ids_180 = self.corner_object_track_ids.get_track_ids("corner180", objs_180)
      for t_id, _, _, _, d_rel, y_rel, v_rel, yv_rel, a_rel in objs_180:
        self._set_corner_pt(t_id, ids_180[t_id], d_rel, y_rel, v_rel, a_rel, yv_rel)

  def _set_corner_pt(self, t_id, track_id, d_rel, y_rel, v_rel, a_rel, yv_rel):
    self.pts[t_id].measured = True
    self.pts[t_id].trackId = track_id
    self.pts[t_id].dRel = d_rel
    self.pts[t_id].yRel = y_rel
    self.pts[t_id].vRel = v_rel
    self.pts[t_id].aRel = a_rel
    self.pts[t_id].yvRel = yv_rel
    self.pts[t_id].vLead = self.v_ego + v_rel

  def _clear_corner_objects(self):
    if self.rcp_corner_235 is not None:
      for slot in range(CORNER_OBJECT_235_MSG_COUNT):
        self.pts[CORNER_OBJECT_235_TRACK_ID_OFFSET + slot].measured = False
      self.corner_object_track_ids.clear_source("corner235")
    if self.rcp_corner_180 is not None:
      for slot in range(CORNER_OBJECT_180_MSG_COUNT * 2):
        self.pts[CORNER_OBJECT_180_TRACK_ID_OFFSET + slot].measured = False
      self.corner_object_track_ids.clear_source("corner180")

  # --- Original Processing Methods ---
  def _update(self, updated_messages):

    t_id = 32
    for addr in range(self.radar_start_addr, self.radar_start_addr + self.radar_msg_count):

      msg = self.rcp_tracks.vl[f"RADAR_TRACK_{addr:x}"]

      if self.radar_group1:
        valid = msg['VALID_CNT1'] > 10
      elif self.canfd:
        valid = msg['VALID_CNT'] > 10
      else:
        valid = msg['STATE'] in (3, 4)

      self.pts[t_id].measured = bool(valid)
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
