from cereal import log
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_MDL
import numpy as np
from openpilot.common.params import Params

from openpilot.selfdrive.controls.lib.desire_lib.constants import (
  LaneChangeState, LaneChangeDirection, TurnDirection,
  LANE_CHANGE_SPEED_MIN, LANE_CHANGE_TIME_MAX,
  BLINKER_NONE, BLINKER_LEFT, BLINKER_RIGHT,
  DESIRES, TURN_DESIRES
)
from openpilot.selfdrive.controls.lib.desire_lib.side_state import SideState
from openpilot.selfdrive.controls.lib.desire_lib.maneuver_classifier import classify_maneuver_type
from openpilot.selfdrive.controls.lib.desire_lib.next_lane import LaneCrossing, StalkPressTracker

STALK_QUEUE_GUARD_S = 0.3
QUEUED_LANE_WAIT_S = 4.0


class DesireHelper:
  def __init__(self):
    self.params = Params()
    self.frame = 0

    # FSM core
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.lane_change_delay = 0.0
    self.maneuver_type = "none"  # "none" / "turn" / "lane_change"

    self.desire = log.Desire.none
    self.turn_direction = TurnDirection.none
    self.enable_turn_desires = True
    self.turn_desire_state = False
    self.desire_disable_count = 0
    self.turn_disable_count = 0

    # per-side states
    self.left = SideState("left")
    self.right = SideState("right")

    # blinker/ATC state
    self.blinker_ignore = False
    self.driver_blinker_state = BLINKER_NONE
    self.carrot_blinker_state = BLINKER_NONE
    self.carrot_lane_change_count = 0
    self.carrot_cmd_index_last = 0
    self.atc_type = ""
    self.atc_active = 0  # 0: 없음, 1: ATC 동작, 2: 충돌

    # auto lane change
    self.auto_lane_change_enable = False
    self.next_lane_change = False

    # keep pulse
    self.keep_pulse_timer = 0.0

    # params
    self.laneChangeNeedTorque = 0
    self.laneChangeBsd = 0
    self.laneLineCheck = 0
    self.laneChangeDelay = 0.0
    self.modelTurnSpeedFactor = 0.0
    self.model_turn_speed = 200.0

    # misc
    self.prev_desire_enabled = False
    self.desireLog = ""

    # externally readable flags
    self.lane_change_available_left = False
    self.lane_change_available_right = False

    # ▼▼▼ [추가] 레버 원터치 판단용 변수 ▼▼▼
    self.stalk_tracker = StalkPressTracker()
    self.stalk_press_side = BLINKER_NONE
    self.stalk_tap_confirmed = False
    self.stalk_queue_eligible = False
    self.stalk_press_timer = 2.0  # 레버 조작 경과 시간(초)
    self.stalk_max_level = 0
    self.one_touch_triggered = False
    self.queued_lane_change = BLINKER_NONE
    self.queued_wait = 0.0
    self.finishing_timer = 0.0
    self.crossing = LaneCrossing()
    self.blinker_hold_direction = LaneChangeDirection.none
    self.handoff_blinker_hold = BLINKER_NONE
    self.followup_requires_press = False
    self.driver_lane_change = False
    self.latched_blinker_state = BLINKER_NONE
    # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

  # ─────────────────────────────────────────────
  # params/model
  # ─────────────────────────────────────────────
  def _update_params_periodic(self):
    if self.frame % 100 == 0:
      self.laneChangeNeedTorque = self.params.get_int("LaneChangeNeedTorque")
      self.laneChangeBsd = self.params.get_int("LaneChangeBsd")
      self.laneLineCheck = self.params.get_int("LaneLineCheck")
      self.laneChangeDelay = self.params.get_float("LaneChangeDelay") * 0.1
      self.modelTurnSpeedFactor = self.params.get_float("ModelTurnSpeedFactor") * 0.1

  def _make_model_turn_speed(self, modeldata):
    if self.modelTurnSpeedFactor > 0:
      model_turn_speed = np.interp(self.modelTurnSpeedFactor,
                                   modeldata.velocity.t,
                                   modeldata.velocity.x) * CV.MS_TO_KPH * 1.2
      self.model_turn_speed = self.model_turn_speed * 0.8 + model_turn_speed * 0.2
    else:
      self.model_turn_speed = 200.0

  def _check_desire_state(self, modeldata, carstate, maneuver_type):
    desire_state = modeldata.meta.desireState
    orientation_rate = abs(modeldata.orientationRate.z[5])
    orientation_rate_future = abs(modeldata.orientationRate.z[15])

    self.turn_desire_state = (desire_state[1] + desire_state[2]) > 0.1

    if maneuver_type == "turn" and abs(carstate.steeringAngleDeg) > 80 and orientation_rate_future < orientation_rate:
      self.turn_disable_count = int(10.0 / DT_MDL)
    else:
      self.turn_disable_count = max(0, self.turn_disable_count - 1)

  # ─────────────────────────────────────────────
  # blinkers/ATC (원본 로직 유지, side 계산은 별개)
  # ─────────────────────────────────────────────
  def _update_driver_blinker(self, carstate):
    st = carstate.leftBlinker * 1 + carstate.rightBlinker * 2
    changed = st != self.driver_blinker_state
    self.driver_blinker_state = st

    # ▼▼▼ 원상 복구: 턴 유도를 위해 체결(2) 신호도 일단 통과시켜 줍니다 ▼▼▼
    enabled = (st in (BLINKER_LEFT, BLINKER_RIGHT)) and (self.one_touch_triggered or carstate.blinkerLever == 2)

    if self.laneChangeNeedTorque < 0:
      enabled = False
    return st, changed, enabled

  def _update_atc_blinker(self, carrotMan, driver_blinker_state):
    atc_type = carrotMan.atcType
    atc_blinker_state = BLINKER_NONE

    # 유지 카운트는 DesireHelper에서 관리
    if self.carrot_lane_change_count > 0:
      atc_blinker_state = self.carrot_blinker_state
    elif carrotMan.carrotCmdIndex != self.carrot_cmd_index_last and carrotMan.carrotCmd == "LANECHANGE":
      self.carrot_cmd_index_last = carrotMan.carrotCmdIndex
      self.carrot_lane_change_count = int(0.2 / DT_MDL)
      self.carrot_blinker_state = BLINKER_LEFT if carrotMan.carrotArg == "LEFT" else BLINKER_RIGHT
      atc_blinker_state = self.carrot_blinker_state
    elif atc_type in ("turn left", "turn right"):
      if self.atc_active != 2:
        atc_blinker_state = BLINKER_LEFT if atc_type == "turn left" else BLINKER_RIGHT
        self.atc_active = 1
        self.blinker_ignore = False
    elif atc_type in ("fork left", "fork right", "atc left", "atc right"):
      if self.atc_active != 2:
        atc_blinker_state = BLINKER_LEFT if atc_type in ("fork left", "atc left") else BLINKER_RIGHT
        self.atc_active = 1
    else:
      self.atc_active = 0

    # 충돌 시 ATC 무효
    if driver_blinker_state != BLINKER_NONE and atc_blinker_state != BLINKER_NONE and driver_blinker_state != atc_blinker_state:
      atc_blinker_state = BLINKER_NONE
      self.atc_active = 2

    atc_desire_enabled = atc_blinker_state in (BLINKER_LEFT, BLINKER_RIGHT)

    # blinker_ignore
    if driver_blinker_state == BLINKER_NONE:
      self.blinker_ignore = False
    if self.blinker_ignore:
      atc_blinker_state = BLINKER_NONE
      atc_desire_enabled = False

    # 타입 변경 1프레임 무시
    if self.atc_type != atc_type:
      atc_desire_enabled = False
    self.atc_type = atc_type

    return atc_blinker_state, atc_desire_enabled

  # ─────────────────────────────────────────────
  # per-side processing (핵심: 좌/우 모두 매 프레임 계산)
  # ─────────────────────────────────────────────
  def _process_sides(self, carstate, modeldata, radarState):
    # geometry (좌/우)
    # left: outer laneLines[0], current laneLines[1], edge[0], cur_prob laneLineProbs[1]
    self.left.update_lane_geometry(
      modeldata.laneLines[0], modeldata.laneLineProbs[0],
      modeldata.laneLines[1],
      modeldata.roadEdges[0],
      cur_prob=modeldata.laneLineProbs[1],
    )
    # right: outer laneLines[3], current laneLines[2], edge[1], cur_prob laneLineProbs[2]
    self.right.update_lane_geometry(
      modeldata.laneLines[3], modeldata.laneLineProbs[3],
      modeldata.laneLines[2],
      modeldata.roadEdges[1],
      cur_prob=modeldata.laneLineProbs[2],
    )

    # lane line info (HUD용 raw는 기존대로 leftLaneLine/rightLaneLine)
    self.left.update_lane_line_info(carstate.leftLaneLine)
    self.right.update_lane_line_info(carstate.rightLaneLine)

    # BSD 설정
    ignore_bsd = (self.laneChangeBsd < 0)

    # obstacles
    v_ego = carstate.vEgo
    self.left.update_obstacles(v_ego, radarState.leadLeft, carstate.leftBlindspot, ignore_bsd, bsd_hold_sec=1.0)
    self.right.update_obstacles(v_ego, radarState.leadRight, carstate.rightBlindspot, ignore_bsd, bsd_hold_sec=1.0)

    # compute available (include BSD+object)
    if self.laneLineCheck >= 1:
      left_line_ok = self.left.lane_line_info_mod in (0, 5)
      right_line_ok = self.right.lane_line_info_mod in (0, 5)
    else:
      left_line_ok = self.left.lane_line_info_raw < 20
      right_line_ok = self.right.lane_line_info_raw < 20
    self.left.compute_lane_change_available(lane_line_info_lt_20=left_line_ok, ignore_bsd=ignore_bsd)
    self.right.compute_lane_change_available(lane_line_info_lt_20=right_line_ok, ignore_bsd=ignore_bsd)

    self.left.update_triggers()
    self.right.update_triggers()

    # externally readable
    self.lane_change_available_left = self.left.lane_change_available
    self.lane_change_available_right = self.right.lane_change_available

  def _get_selected_side(self, blinker_state: int) -> SideState:
    return self.left if blinker_state == BLINKER_LEFT else self.right

  def _latched_turn_side(self, carstate, stalk_press, raw_blinker):
    """Resolve a manual latch from physical input, never a navigation request."""
    if carstate.blinkerLever != 2:
      self.latched_blinker_state = BLINKER_NONE
      return BLINKER_NONE
    if self.stalk_tracker.invalidated:
      self.latched_blinker_state = BLINKER_NONE
      return BLINKER_NONE
    if stalk_press:
      self.latched_blinker_state = stalk_press
    elif not self.stalk_tap_confirmed and self.stalk_press_side:
      self.latched_blinker_state = self.stalk_press_side
    elif not self.latched_blinker_state and raw_blinker in (BLINKER_LEFT, BLINKER_RIGHT):
      self.latched_blinker_state = raw_blinker
    self.stalk_tap_confirmed = True
    return BLINKER_NONE if raw_blinker == 3 else self.latched_blinker_state

  # ─────────────────────────────────────────────
  # main update
  # ─────────────────────────────────────────────
  def update(self, carstate, modeldata, lateral_active, lane_change_prob, carrotMan, radarState):
    self.frame += 1
    self._update_params_periodic()
    self._make_model_turn_speed(modeldata)

    # ▼▼▼ [수정] 깜빡이 레버 물리 상태(Debounce) 추적 로직 ▼▼▼
    previous_state = self.lane_change_state
    stalk_press = self.stalk_tracker.update(carstate)
    confirmed_tap = BLINKER_NONE
    if self.stalk_tracker.invalidated:
      self.stalk_press_side = BLINKER_NONE
      self.stalk_tap_confirmed = True
      self.stalk_max_level = 0
      self.one_touch_triggered = False
      self.followup_requires_press = True
      self.stalk_queue_eligible = False

    # 새로운 레버 조작(Count 변화)이 감지되면 타이머 및 상태 갱신
    if stalk_press != BLINKER_NONE:
      self.stalk_press_timer = 0.0
      # A complete tap can fall between two model frames. A progressing physical
      # press counter with an already released lever still represents that tap.
      self.stalk_max_level = 1 if carstate.blinkerLever == 0 else carstate.blinkerLever
      self.stalk_press_side = stalk_press
      self.stalk_tap_confirmed = False
      self.stalk_queue_eligible = False
      if self.lane_change_state == LaneChangeState.off:
        self.one_touch_triggered = False

    # 레버가 도달한 최대 깊이(1: 원터치, 2: 완전 체결) 갱신
    if carstate.blinkerLever > self.stalk_max_level:
      self.stalk_max_level = carstate.blinkerLever

    if self.stalk_press_timer < 2.0:
      self.stalk_press_timer += DT_MDL

    # A latch passes through detent 1. Waiting for release avoids starting a
    # lane change while a slow movement toward detent 2 is still undecided.
    tap_confirmed = carstate.blinkerLever == 0 and self.stalk_max_level == 1 and self.stalk_press_timer > 0
    if tap_confirmed:
      self.one_touch_triggered = True
    if tap_confirmed and not self.stalk_tap_confirmed:
      confirmed_tap = self.stalk_press_side
      self.stalk_tap_confirmed = True

    # 완전 체결(2)이 감지되었다면 원터치 동작은 무조건 취소 (메뉴얼 턴/수동 조향용)
    if self.stalk_max_level == 2:
      self.one_touch_triggered = False

    # 💡 [핵심 수정] 깜빡이 램프가 모두 꺼졌을 때의 처리
    # 차선 변경 로직이 완전히 종료(off)된 상태에서 깜빡이가 꺼져 있어야만 트리거 해제
    # (차선 변경 도중에 깜빡이가 먼저 꺼져도, FSM이 돌아가고 있다면 트리거 유지)
    if carstate.leftBlinker == 0 and carstate.rightBlinker == 0:
      if self.lane_change_state == LaneChangeState.off:
        self.one_touch_triggered = False
    # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

    # counts
    self.carrot_lane_change_count = max(0, self.carrot_lane_change_count - 1)
    self.lane_change_delay = max(0.0, self.lane_change_delay - DT_MDL)

    v_ego = carstate.vEgo
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    raw_blinker = int(carstate.leftBlinker) + 2 * int(carstate.rightBlinker)
    manual_latched = carstate.blinkerLever == 2
    manual_turn_side = self._latched_turn_side(carstate, stalk_press, raw_blinker)
    request_allowed = (lateral_active and carstate.canValid and not below_lane_change_speed
                       and self.laneChangeNeedTorque >= 0 and self.desire_disable_count == 0)
    if request_allowed and confirmed_tap:
      self.followup_requires_press = False
    elif not request_allowed:
      self.followup_requires_press = True
    if previous_state in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing):
      self.crossing.update(modeldata, self.lane_change_direction == LaneChangeDirection.left)
      current_side = BLINKER_LEFT if self.lane_change_direction == LaneChangeDirection.left else BLINKER_RIGHT
      after_crossing = (self.crossing.since >= 0.0 or
                        (previous_state == LaneChangeState.laneChangeFinishing
                         and self.finishing_timer + DT_MDL >= STALK_QUEUE_GUARD_S))
      if stalk_press:
        # Classify intent at the press, not its later release. Holding an early
        # press through the crossing must not promote it into a next-lane request.
        self.stalk_queue_eligible = request_allowed and stalk_press == current_side and after_crossing
      if request_allowed and confirmed_tap == current_side and self.stalk_queue_eligible:
        self.queued_lane_change = confirmed_tap
        self.queued_wait = 0.0
    elif previous_state == LaneChangeState.preLaneChange:
      if stalk_press:
        self.stalk_queue_eligible = request_allowed and self.next_lane_change
      if self.next_lane_change and request_allowed and confirmed_tap and self.stalk_queue_eligible:
        # The first change already finished; accept a new tap even when the old
        # lamp has not gone dark. It must pass the normal preLaneChange gates.
        self.queued_lane_change = confirmed_tap
        self.queued_wait = 0.0
        self.next_lane_change = False

    queued = self.queued_lane_change
    incompatible = (self.stalk_tracker.invalidated or raw_blinker == 3 or carstate.blinkerLever == 2
                    or (queued and raw_blinker not in (BLINKER_NONE, queued))
                    or (queued and stalk_press not in (BLINKER_NONE, queued)))
    if previous_state == LaneChangeState.preLaneChange and queued:
      self.queued_wait += DT_MDL
    if (not request_allowed or incompatible or
        (previous_state == LaneChangeState.preLaneChange and self.queued_wait >= QUEUED_LANE_WAIT_S)):
      if queued:
        self.next_lane_change = True  # A later BSD/line release cannot revive an expired request.
        self.followup_requires_press = True
      self.queued_lane_change = BLINKER_NONE
    if (not request_allowed or self.stalk_tracker.invalidated or raw_blinker == 3 or carstate.blinkerLever == 2
        or (self.handoff_blinker_hold and
            (raw_blinker not in (BLINKER_NONE, self.handoff_blinker_hold)
             or stalk_press not in (BLINKER_NONE, self.handoff_blinker_hold)))):
      self.handoff_blinker_hold = BLINKER_NONE
    if self.crossing.since >= 0.0:
      self.handoff_blinker_hold = BLINKER_NONE

    # per-side compute (좌/우 모두)
    self._process_sides(carstate, modeldata, radarState)

    # desire state from model
    self._check_desire_state(modeldata, carstate, self.maneuver_type)

    # blinkers
    driver_st, driver_changed, driver_enabled = self._update_driver_blinker(carstate)
    if self.queued_lane_change:
      # The physical one-touch flashes can end before the model finishes this
      # change. Keep the explicit queued request, bounded below, independently.
      driver_st = self.queued_lane_change
      driver_enabled = True
    atc_st, atc_enabled = self._update_atc_blinker(carrotMan, driver_st)

    desire_enabled = driver_enabled or atc_enabled
    blinker_state = driver_st if driver_enabled else atc_st

    # 선택된 side (FSM은 이 side만 참고)
    side = self._get_selected_side(blinker_state) if blinker_state in (BLINKER_LEFT, BLINKER_RIGHT) else None
    atc_lane_change_requested = (
      atc_enabled and
      self.atc_type in ("fork left", "fork right", "atc left", "atc right")
    )
    atc_lane_change_manual_only = (
      atc_enabled and
      not driver_enabled and
      self.atc_type in ("fork left", "atc left")
    )
    atc_lane_change_only = atc_lane_change_requested and not driver_enabled
    # Do not treat a blocked->available retry as permission to cross solid/unknown lines.
    # Geometry-based ATC at the last lane still works when the lane/road edge opens up.
    atc_lane_change_retry_line_blocked = (
      atc_lane_change_only and
      side is not None and
      side.lane_line_info_mod not in (0, 5)
    )

    # auto lane change trigger (기존 로직 유지하되 side 기반)
    auto_lane_change_trigger = False
    if desire_enabled and side is not None:
      # carrot_lane_change_count>0이면 강제 허용
      if self.carrot_lane_change_count > 0:
        auto_lane_change_trigger = side.lane_change_available
      else:
        # 기존 조건: edge_available + (trigger or appeared) + not side_object_detected
        auto_lane_change_trigger = (
          self.auto_lane_change_enable and
          (not atc_lane_change_manual_only) and
          side.edge_available and
          (side.lane_available_trigger or side.lane_appeared) and
          (not side.side_object_detected) and
          (side.bsd_hold_counter == 0)
        )
      self.desireLog = (
        f"{side.name}:ALC={self.auto_lane_change_enable}, "
        #f"L={side.lane_available},E={side.edge_available}, "
        #f"T={side.lane_available_trigger},A={side.lane_appeared}, "
        #f"OBJ={side.side_object_detected},BSD={side.bsd_hold_counter>0}"
      )
    else:
      self.auto_lane_change_enable = False
      self.next_lane_change = False

    # ───────────────────────── FSM ─────────────────────────
    if (not lateral_active or not carstate.canValid
        or (not manual_latched and self.lane_change_timer > LANE_CHANGE_TIME_MAX)):
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.turn_direction = TurnDirection.none
      self.maneuver_type = "none"
      self.followup_requires_press = True

    elif self.desire_disable_count > 0:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.turn_direction = TurnDirection.none
      self.maneuver_type = "none"
      self.followup_requires_press = True

    elif manual_latched:
      # A fully latched lever is a manual turn input in every lane-change
      # state. It cannot inherit a pending tap or authorize an ATC lane change.
      was_turn_active = self.turn_direction != TurnDirection.none
      self.maneuver_type = "turn"
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.turn_direction = TurnDirection.none
      self.lane_change_ll_prob = 1.0
      self.lane_change_timer = self.lane_change_delay = self.finishing_timer = 0.0
      self.queued_lane_change = self.handoff_blinker_hold = BLINKER_NONE
      self.queued_wait = 0.0
      self.next_lane_change = self.auto_lane_change_enable = False
      self.followup_requires_press = True
      self.stalk_queue_eligible = False
      self.driver_lane_change = False
      if manual_turn_side in (BLINKER_LEFT, BLINKER_RIGHT):
        turn_side = self._get_selected_side(manual_turn_side)
        # Classification as TURN does not enable new high-speed turn steering.
        # Retain the existing speed/geometry/navigation eligibility and cooldown.
        turn_type = classify_maneuver_type(
          blinker_state=manual_turn_side, carstate=carstate, side=turn_side,
          turn_desire_state=self.turn_desire_state, atc_type=self.atc_type,
          old_type="turn" if was_turn_active else "none",
        )
        if self.enable_turn_desires and self.turn_disable_count == 0 and turn_type == "turn":
          self.turn_direction = TurnDirection.turnLeft if manual_turn_side == BLINKER_LEFT else TurnDirection.turnRight
          self.lane_change_direction = self.turn_direction

    else:
      # classify maneuver type using selected side
      if desire_enabled and side is not None:
        new_type = classify_maneuver_type(
          blinker_state=blinker_state,
          carstate=carstate,
          side=side,
          turn_desire_state=self.turn_desire_state,
          atc_type=self.atc_type,
          old_type=self.maneuver_type,
        )
      else:
        new_type = "none"

      # switching rules
      if self.maneuver_type == "lane_change" and new_type == "turn" and self.lane_change_state not in (
        LaneChangeState.preLaneChange, LaneChangeState.laneChangeStarting
      ):
        self.maneuver_type = "turn"
        self.lane_change_state = LaneChangeState.off
      elif self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
        self.maneuver_type = new_type

      # ─ TURN mode ─
      if desire_enabled and self.maneuver_type == "turn" and self.enable_turn_desires:
        self.lane_change_state = LaneChangeState.off
        if self.turn_disable_count > 0:
          self.turn_direction = TurnDirection.none
          self.lane_change_direction = LaneChangeDirection.none
        else:
          self.turn_direction = TurnDirection.turnLeft if blinker_state == BLINKER_LEFT else TurnDirection.turnRight
          self.lane_change_direction = self.turn_direction

      # ─ Lane change FSM ─
      else:
        self.turn_direction = TurnDirection.none

        if self.lane_change_state == LaneChangeState.off:
          fresh_driver_request = not driver_enabled or not self.followup_requires_press or confirmed_tap
          if (desire_enabled and (not self.prev_desire_enabled or confirmed_tap) and fresh_driver_request
              and carstate.blinkerLever == 0 and not below_lane_change_speed and side is not None):
            # ▼▼▼ [수정] 원터치 조작이거나, 자동차선변경(ATC)일 때만 차선변경 로직 진입 허용 ▼▼▼
            if self.one_touch_triggered or atc_enabled or self.carrot_lane_change_count > 0:
              self.lane_change_state = LaneChangeState.preLaneChange
              self.lane_change_ll_prob = 1.0
              self.lane_change_delay = self.laneChangeDelay
  
              # 맨 끝 차선이 아니면, ATC 자동 차선변경 비활성
              # (원본 유지: 차선 존재하거나 geom 가능하면 auto off, 아니면 on)
              lane_exist_counter_side = side.lane_exist_count.counter
              lane_change_available_geom = side.lane_change_available_geom
              self.auto_lane_change_enable = False if (lane_exist_counter_side > 0 or lane_change_available_geom) else True
              self.next_lane_change = False

        elif self.lane_change_state == LaneChangeState.preLaneChange:
          if side is None:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none
          else:
            self.lane_change_direction = LaneChangeDirection.left if blinker_state == BLINKER_LEFT else LaneChangeDirection.right

            # torque direction cond
            torque_cond = (carstate.steeringTorque > 0) if blinker_state == BLINKER_LEFT else (carstate.steeringTorque < 0)
            torque_applied = carstate.steeringPressed and torque_cond

            # BSD config
            ignore_bsd = (self.laneChangeBsd < 0)
            block_lanechange_bsd = (self.laneChangeBsd == 1)
            bsd_active = (side.bsd_hold_counter > 0) and (not ignore_bsd)
            side_clear_without_line = (side.lane_available or side.edge_available) and \
                                      (not side.side_object_detected) and (not bsd_active)
            atc_driver_confirm = atc_lane_change_requested and driver_enabled
            atc_geometry_release = atc_lane_change_only and auto_lane_change_trigger
            atc_line_release = (atc_driver_confirm or atc_geometry_release) and side_clear_without_line

            if not desire_enabled or below_lane_change_speed:
              self.lane_change_state = LaneChangeState.off
              self.lane_change_direction = LaneChangeDirection.none
            else:
              # 차선변경 시작 조건:
              solid_line_blocked = (self.laneLineCheck >= 2) and (not side.lane_change_available_geom) and \
                                   (side.lane_available or side.edge_available)
              block_released = side.lane_change_available_released
              block_released_auto = block_released and not atc_lane_change_retry_line_blocked
              start_gate = (side.lane_change_available_geom and self.lane_change_delay == 0) or \
                           side.lane_line_info_edge_detect or solid_line_blocked or block_released_auto or atc_line_release

              queued_signal_ready = not self.queued_lane_change or raw_blinker == self.queued_lane_change
              if start_gate and queued_signal_ready and carstate.blinkerLever == 0:
                if solid_line_blocked:
                  if atc_line_release or (torque_applied and not (bsd_active and block_lanechange_bsd)):
                    self.lane_change_state = LaneChangeState.laneChangeStarting
                elif bsd_active:
                  if torque_applied and (not block_lanechange_bsd):
                    self.lane_change_state = LaneChangeState.laneChangeStarting
                elif self.laneChangeNeedTorque > 0 or self.next_lane_change:
                  if torque_applied:
                    self.lane_change_state = LaneChangeState.laneChangeStarting
                elif driver_enabled:
                  # driver blinker면 바로 시작(원본 유지)
                  # 단, object/bzd 막힘은 side.lane_change_available에서 걸림
                  if side.lane_change_available or atc_line_release:
                    self.lane_change_state = LaneChangeState.laneChangeStarting
                else:
                  if torque_applied or ((not atc_lane_change_manual_only) and (
                    auto_lane_change_trigger or side.lane_line_info_edge_detect or block_released_auto
                  )):
                    # 여기서는 시작 직전 안전성 체크
                    if side.lane_change_available or atc_line_release:
                      self.lane_change_state = LaneChangeState.laneChangeStarting

        elif self.lane_change_state == LaneChangeState.laneChangeStarting:
          self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)
          if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
            self.lane_change_state = LaneChangeState.laneChangeFinishing
            self.finishing_timer = 0.0

        elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
          self.finishing_timer += DT_MDL
          self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)
          if self.lane_change_ll_prob > 0.99:
            self.lane_change_direction = LaneChangeDirection.none
            if desire_enabled:
              self.lane_change_state = LaneChangeState.preLaneChange
              # A lingering lamp is not a second automatic request. A fresh
              # queued tap removes only this extra confirmation, never the
              # configured LaneChangeNeedTorque/BSD/line checks above.
              self.next_lane_change = driver_enabled and self.queued_lane_change != blinker_state
              self.queued_wait = 0.0
              if self.driver_lane_change and not self.queued_lane_change:
                self.followup_requires_press = True
            else:
              self.lane_change_state = LaneChangeState.off
              if self.driver_lane_change:
                self.followup_requires_press = True
    
    # timer
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    # commit last per-side
    self.left.commit_last()
    self.right.commit_last()

    self.prev_desire_enabled = desire_enabled

    # 반대 방향 토크로 cancel (기존 유지)
    steering_pressed_cancel = carstate.steeringPressed and (
      (carstate.steeringTorque < 0 and blinker_state == BLINKER_LEFT) or
      (carstate.steeringTorque > 0 and blinker_state == BLINKER_RIGHT)
    )
    if steering_pressed_cancel and self.lane_change_state != LaneChangeState.off:
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_state = LaneChangeState.off
      self.blinker_ignore = True
      self.followup_requires_press = True

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.crossing.reset()
    if self.lane_change_state == LaneChangeState.laneChangeStarting and previous_state == LaneChangeState.preLaneChange:
      self.driver_lane_change = driver_enabled
      # Keep the queued signal through the new maneuver's initial model-probability
      # ramp. The legacy hold can be zero here, after the one-touch flashes ended.
      self.handoff_blinker_hold = self.queued_lane_change
      self.queued_lane_change = BLINKER_NONE
    elif self.lane_change_state == LaneChangeState.off:
      self.queued_lane_change = BLINKER_NONE
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.handoff_blinker_hold = BLINKER_NONE
    hold_side = self.queued_lane_change or self.handoff_blinker_hold
    self.blinker_hold_direction = {
      BLINKER_LEFT: LaneChangeDirection.left, BLINKER_RIGHT: LaneChangeDirection.right,
    }.get(hold_side, LaneChangeDirection.none)

    # final desire
    if self.turn_direction != TurnDirection.none:
      self.desire = TURN_DESIRES[self.turn_direction]
      self.lane_change_direction = self.turn_direction
    else:
      self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # keep pulse
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none

    return self.desire
