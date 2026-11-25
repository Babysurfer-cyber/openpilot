from cereal import log
import math
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_MDL
import numpy as np
from openpilot.common.params import Params

from openpilot.selfdrive.controls.lib.desire_lib.constants import (
  LaneChangeState, LaneChangeDirection, TurnDirection,
  LANE_CHANGE_SPEED_MIN,
  BLINKER_NONE, BLINKER_LEFT, BLINKER_RIGHT,
  DESIRES, TURN_DESIRES
)
from openpilot.selfdrive.controls.lib.desire_lib.side_state import SideState
from openpilot.selfdrive.controls.lib.desire_lib.maneuver_classifier import classify_maneuver_type


# 모델 기준점과 차선 사이에 여유가 있는 초기 구간에서만 BSD 취소를 허용합니다.
# 아래 수치는 차체의 실제 차선 통과를 측정하는 값이 아닌 보수적 판정 기준입니다.
BSD_ABORT_MIN_DISTANCE = 0.9
BSD_ABORT_MAX_DISTANCE = 2.4
BSD_ABORT_MIN_LINE_PROB = 0.5
BSD_ABORT_MAX_DISTANCE_STEP = 0.35
BSD_ABORT_MAX_FRAME_GAP_NS = 150_000_000

# 업로드한 EV6 로그의 램프 점멸 주기 약 0.7초 × 5회에 해당하는 시간입니다.
LANE_CHANGE_TIME_MAX = 0.7 * 5


class ManualBoundaryCrossing:
  MIN_LINE_PROB = 0.5
  MAX_DISTANCE_STEP = 0.35
  MAX_FRAME_GAP_NS = 150_000_000
  CROSS_MARGIN = 0.2
  CONFIRM_FRAMES = 2

  def __init__(self):
    self.reset()

  def reset(self):
    self.direction = None
    self.previous_y = None
    self.timestamp_ns = 0
    self.crossed_frames = 0
    self.active = False

  @staticmethod
  def _timestamp(modeldata):
    try:
      timestamp = int(getattr(modeldata, "timestampEof", 0))
    except (TypeError, ValueError, OverflowError):
      return 0
    return timestamp if timestamp > 0 else 0

  @classmethod
  def _line_y(cls, modeldata, index):
    lines = getattr(modeldata, "laneLines", ())
    probabilities = getattr(modeldata, "laneLineProbs", ())
    if len(lines) <= index or len(probabilities) <= index or not len(lines[index].y):
      return None
    probability, y = probabilities[index], lines[index].y[0]
    if not (math.isfinite(probability) and probability > cls.MIN_LINE_PROB and math.isfinite(y)):
      return None
    return float(y)

  def begin(self, modeldata, direction):
    """Start only for a new pending request; direction is 'left' or 'right'."""
    self.reset()
    if direction not in ("left", "right"):
      return
    self.direction = direction
    self.timestamp_ns = self._timestamp(modeldata)
    self.previous_y = self._line_y(modeldata, 1 if direction == "left" else 2)
    side_sign = -1 if direction == "left" else 1
    self.active = bool(self.timestamp_ns and self.previous_y is not None and side_sign * self.previous_y > 0.0)

  def update(self, modeldata, direction):
    """Return True once after two continuous observations beyond the boundary."""
    if not self.active:
      return False
    timestamp = self._timestamp(modeldata)
    if direction != self.direction or not 0 < timestamp - self.timestamp_ns <= self.MAX_FRAME_GAP_NS:
      self.active = False
      return False

    # The same physical boundary can move to another model lane-line index.
    candidates = [y for index in range(4) if (y := self._line_y(modeldata, index)) is not None]
    y = min(candidates, key=lambda value: abs(value - self.previous_y), default=None)
    if y is None or abs(y - self.previous_y) > self.MAX_DISTANCE_STEP:
      self.active = False
      return False
    self.previous_y, self.timestamp_ns = y, timestamp
    side_sign = -1 if self.direction == "left" else 1
    self.crossed_frames = self.crossed_frames + 1 if side_sign * y <= -self.CROSS_MARGIN else 0
    if self.crossed_frames >= self.CONFIRM_FRAMES:
      self.active = False
      return True
    return False


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
    self.lane_change_blocked = 0  # 0: none, 1: BSD, 2: lane line

    # ▼▼▼ [추가] 레버 원터치 판단용 변수 ▼▼▼
    self.prev_left_stalk_count = 0
    self.prev_right_stalk_count = 0
    self.stalk_press_timer = 2.0  # 초기값은 동작 시간(0.15초)을 넘겨서 세팅
    self.stalk_max_level = 0
    self.one_touch_triggered = False
    # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

    # BSD로 취소한 입력은 새로운 레버 조작/ATC 요청 전까지 다시 사용하지 않습니다.
    self.bsd_cancelled = False
    self.bsd_stalk_cancelled = False
    self.bsd_abort_allowed = False
    self.bsd_abort_direction = LaneChangeDirection.none
    self.bsd_abort_distance = None
    self.bsd_abort_timestamp = 0

    # 완료된 원터치는 새 레버 조작 전까지 재사용하지 않습니다.
    self.stalk_direction = BLINKER_NONE
    self.one_touch_completed = False
    self.manual_crossing = ManualBoundaryCrossing()
    self.manual_crossed = False
    self.held_crossing = ManualBoundaryCrossing()
    self.held_crossed = False
    self.held_model_seen = False
    self.held_repeat_pending = False
    self.lane_change_driver_request = False
    self.lane_change_stalk_counts = None

  def _one_touch_held(self, carstate):
    return bool(carstate.blinkerLever == 1 and self.stalk_max_level == 1
                and self.stalk_direction in (BLINKER_LEFT, BLINKER_RIGHT)
                and self.one_touch_triggered and not self.one_touch_completed
                and not self.bsd_stalk_cancelled)

  def _consume_lane_change_request(self, carrotMan, reason, bsd=False, preserve_new_stalk=False, carstate=None):
    current_counts = (self.prev_left_stalk_count, self.prev_right_stalk_count)
    preserve = (preserve_new_stalk and carstate is not None and self.lane_change_stalk_counts is not None
                and current_counts != self.lane_change_stalk_counts)
    saved_level, saved_timer = self.stalk_max_level, self.stalk_press_timer
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.turn_direction = TurnDirection.none
    self.maneuver_type = "none"
    self.lane_change_ll_prob = 1.0
    self.one_touch_triggered = False
    self.one_touch_completed = True
    self.bsd_cancelled = True
    self.bsd_stalk_cancelled = self.bsd_stalk_cancelled or bsd
    self.stalk_max_level = 0
    self.stalk_press_timer = 2.0
    self.blinker_ignore = True
    self.carrot_lane_change_count = 0
    self.carrot_blinker_state = BLINKER_NONE
    self.carrot_cmd_index_last = carrotMan.carrotCmdIndex
    self.auto_lane_change_enable = False
    self.next_lane_change = False
    self.manual_crossing.reset()
    self.manual_crossed = False
    self.held_repeat_pending = False
    self.desireLog = reason
    # 이전 1회 완료 시점에 이미 들어온 새 원터치는 그 이전 요청과 함께 지우지 않습니다.
    if preserve:
      self.one_touch_completed = self.bsd_stalk_cancelled = False
      self.stalk_max_level, self.stalk_press_timer = saved_level, saved_timer
      self.one_touch_triggered = bool(saved_level == 1 and (carstate.blinkerLever == 0 or saved_timer >= 0.3))

  def _held_next_lane_ready(self, carstate, modeldata):
    if (self.laneChangeNeedTorque < 0 or not carstate.canValid or not math.isfinite(carstate.vEgo)
        or carstate.vEgo < LANE_CHANGE_SPEED_MIN):
      return False
    side = self.left if self.lane_change_direction == LaneChangeDirection.left else self.right
    torque_sign = 1 if self.lane_change_direction == LaneChangeDirection.left else -1
    torque_applied = carstate.steeringPressed and carstate.steeringTorque * torque_sign > 0
    if self.laneChangeNeedTorque > 0 and not torque_applied:
      return False
    if not side.lane_change_available_geom or side.side_object_detected:
      return False
    # 다음 차로도 BSD와 원래의 차선 허용 조건을 다시 만족해야 새 요청을 보냅니다.
    if self.laneChangeBsd >= 0 and side.bsd_hold_counter > 0:
      if self.laneChangeBsd == 1 or not torque_applied:
        return False
    return self._update_bsd_abort_window(modeldata, new_start=True)

  def _update_bsd_abort_window(self, modeldata, new_start=False):
    direction = self.lane_change_direction
    index = 1 if direction == LaneChangeDirection.left else 2 if direction == LaneChangeDirection.right else -1
    timestamp = int(getattr(modeldata, "timestampEof", 0))
    distance = None
    if index >= 0 and len(modeldata.laneLines) > index and len(modeldata.laneLineProbs) > index:
      line = modeldata.laneLines[index]
      probability = modeldata.laneLineProbs[index]
      if len(line.y) > 0 and math.isfinite(probability) and probability > BSD_ABORT_MIN_LINE_PROB:
        # 모델 좌표는 왼쪽 음수, 오른쪽 양수입니다.
        y = line.y[0]
        value = -y if direction == LaneChangeDirection.left else y
        if math.isfinite(value) and BSD_ABORT_MIN_DISTANCE < value <= BSD_ABORT_MAX_DISTANCE:
          distance = float(value)

    if new_start:
      self.bsd_abort_allowed = distance is not None and timestamp > 0
      self.bsd_abort_direction = direction
    else:
      # 가까워짐·인식 소실·좌표 점프·수신 공백이 있으면 이번 변경에서는 다시 열지 않습니다.
      self.bsd_abort_allowed = bool(
        self.bsd_abort_allowed and direction == self.bsd_abort_direction
        and distance is not None and self.bsd_abort_distance is not None
        and abs(distance - self.bsd_abort_distance) <= BSD_ABORT_MAX_DISTANCE_STEP
        and 0 < timestamp - self.bsd_abort_timestamp <= BSD_ABORT_MAX_FRAME_GAP_NS
      )
    self.bsd_abort_distance = distance
    self.bsd_abort_timestamp = timestamp
    return self.bsd_abort_allowed

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

  def _update_lane_change_blocked(self, carstate, side, driver_enabled, blinker_state):
    # Export a wait reason only for a released physical one-touch request.
    # This observes the final FSM state and does not change its start gates.
    if (self.lane_change_state != LaneChangeState.preLaneChange or side is None
        or not driver_enabled or not self.one_touch_triggered or self.one_touch_completed
        or self.bsd_stalk_cancelled or carstate.blinkerLever != 0 or self.stalk_max_level != 1
        or self.stalk_direction != blinker_state
        or blinker_state not in (BLINKER_LEFT, BLINKER_RIGHT)
        or carstate.vEgo < LANE_CHANGE_SPEED_MIN
        or not (side.lane_available or side.edge_available)
        or self.lane_change_delay > 0):
      return

    torque_cond = carstate.steeringTorque > 0 if blinker_state == BLINKER_LEFT else carstate.steeringTorque < 0
    torque_applied = carstate.steeringPressed and torque_cond
    if (self.laneChangeNeedTorque > 0 or self.next_lane_change) and not torque_applied:
      return
    bsd_blocked = (self.laneChangeBsd >= 0 and side.bsd_hold_counter > 0
                   and (self.laneChangeBsd == 1 or not torque_applied))
    if bsd_blocked:
      self.lane_change_blocked = 1
    elif not side.side_object_detected:
      line_blocked = (side.lane_line_info_mod not in (0, 5) if self.laneLineCheck >= 1
                      else side.lane_line_info_raw >= 20)
      if line_blocked and not (self.laneLineCheck >= 2 and torque_applied):
        self.lane_change_blocked = 2

  # ─────────────────────────────────────────────
  # main update
  # ─────────────────────────────────────────────
  def update(self, carstate, modeldata, lateral_active, lane_change_prob, carrotMan, radarState):
    self.lane_change_blocked = 0
    previous_lane_change_state = self.lane_change_state
    self.frame += 1
    self._update_params_periodic()
    self._make_model_turn_speed(modeldata)

    # ▼▼▼ [수정] 깜빡이 레버 물리 상태(Debounce) 추적 로직 ▼▼▼
    left_stalk_changed = carstate.leftBlinkerStalkCount != self.prev_left_stalk_count
    right_stalk_changed = carstate.rightBlinkerStalkCount != self.prev_right_stalk_count

    # 새로운 레버 조작(Count 변화)이 감지되면 타이머 및 상태 갱신
    if left_stalk_changed or right_stalk_changed:
      self.bsd_cancelled = False
      self.bsd_stalk_cancelled = False
      self.one_touch_completed = False
      self.stalk_direction = BLINKER_LEFT if left_stalk_changed and not right_stalk_changed else \
                             BLINKER_RIGHT if right_stalk_changed and not left_stalk_changed else BLINKER_NONE
      self.manual_crossing.reset()
      self.manual_crossed = False
      self.prev_left_stalk_count = carstate.leftBlinkerStalkCount
      self.prev_right_stalk_count = carstate.rightBlinkerStalkCount
      self.stalk_press_timer = 0.0
      self.stalk_max_level = 0
      
      # 💡 핵심: 톡 칠 때마다 즉시 다시 트리거를 살림 (연속 차선 변경 허용)
      if self.lane_change_state != LaneChangeState.off:
          self.one_touch_triggered = True
      else:
          self.one_touch_triggered = False

    # 레버가 도달한 최대 깊이(1: 원터치, 2: 완전 체결) 갱신
    if carstate.blinkerLever > self.stalk_max_level:
      self.stalk_max_level = carstate.blinkerLever

    if self.stalk_press_timer < 2.0:
      self.stalk_press_timer += DT_MDL

    # 원터치 확정 로직 (0.3초 안에 완전 체결이 되지 않고 1까지만 갔다면 원터치)
    if self.stalk_max_level == 1 and self.stalk_press_timer >= 0.3:
      self.one_touch_triggered = True
    elif carstate.blinkerLever == 0 and self.stalk_max_level == 1 and self.stalk_press_timer > 0:
      # 0.3초 전에 레버가 다시 0으로 튕겨 올라왔어도 원터치로 확정
      self.one_touch_triggered = True

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

    new_atc_command = (carrotMan.carrotCmd == "LANECHANGE"
                       and carrotMan.carrotCmdIndex != self.carrot_cmd_index_last)
    new_atc_maneuver = (carrotMan.atcType != self.atc_type and carrotMan.atcType in
                        ("turn left", "turn right", "fork left", "fork right", "atc left", "atc right"))
    if new_atc_command or new_atc_maneuver:
      self.bsd_cancelled = False
    # 새로운 ATC 요청이 와도 취소된 물리 레버 입력은 새 레버 조작 전까지 소비 상태입니다.
    if self.bsd_stalk_cancelled or self.one_touch_completed:
      self.one_touch_triggered = False

    # counts
    self.carrot_lane_change_count = max(0, self.carrot_lane_change_count - 1)
    self.lane_change_delay = max(0.0, self.lane_change_delay - DT_MDL)

    v_ego = carstate.vEgo
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    # per-side compute (좌/우 모두)
    self._process_sides(carstate, modeldata, radarState)

    # desire state from model
    self._check_desire_state(modeldata, carstate, self.maneuver_type)

    # blinkers
    driver_st, driver_changed, driver_enabled = self._update_driver_blinker(carstate)
    # 1단을 실제로 누르는 중에는 램프의 순간 소등과 무관하게 그 방향 입력을 유지합니다.
    if self._one_touch_held(carstate) and driver_st == BLINKER_NONE:
      driver_st = self.stalk_direction
      driver_enabled = self.laneChangeNeedTorque >= 0
    atc_st, atc_enabled = self._update_atc_blinker(carrotMan, driver_st)

    # blinker_ignore만으로는 운전자 원터치 입력을 차단할 수 없습니다.
    if (self.bsd_stalk_cancelled or self.one_touch_completed) and carstate.blinkerLever != 2:
      driver_enabled = False
    if self.bsd_cancelled:
      atc_enabled = False

    desire_enabled = driver_enabled or atc_enabled
    blinker_state = driver_st if driver_enabled else atc_st

    # 선택된 side (FSM은 이 side만 참고)
    side = self._get_selected_side(blinker_state) if blinker_state in (BLINKER_LEFT, BLINKER_RIGHT) else None

    # 자동 시작 전 수동 횡단도 같은 원터치의 1회 완료로 처리합니다.
    pending_driver = (self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange)
                      and self.one_touch_triggered and not self.one_touch_completed
                      and carstate.blinkerLever != 2
                      and self.stalk_direction in (BLINKER_LEFT, BLINKER_RIGHT))
    if pending_driver:
      direction = "left" if self.stalk_direction == BLINKER_LEFT else "right"
      if self.manual_crossing.direction is None:
        self.manual_crossing.begin(modeldata, direction)
      else:
        self.manual_crossed = self.manual_crossing.update(modeldata, direction) or self.manual_crossed
      # 계속 누르고 있는 입력은 다음 차로까지 진행하려는 의도이므로 놓을 때만 소비합니다.
      if self.manual_crossed and not self._one_touch_held(carstate):
        self._consume_lane_change_request(carrotMan, "Manual lane change completed")
        driver_enabled = atc_enabled = desire_enabled = False
        blinker_state, side = BLINKER_NONE, None
    elif self.lane_change_state not in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.manual_crossing.reset()
      self.manual_crossed = False
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
    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.turn_direction = TurnDirection.none
      self.maneuver_type = "none"
      if self.lane_change_timer > LANE_CHANGE_TIME_MAX and (self.one_touch_triggered or self.lane_change_driver_request):
        self._consume_lane_change_request(carrotMan, "Lane change timeout")
        desire_enabled = False

    elif self.desire_disable_count > 0:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.turn_direction = TurnDirection.none
      self.maneuver_type = "none"

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
          if desire_enabled and not self.prev_desire_enabled and not below_lane_change_speed and side is not None:
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

              if start_gate:
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
          direction = "left" if self.lane_change_direction == LaneChangeDirection.left else "right"
          held_same_direction = (self._one_touch_held(carstate)
                                 and self.stalk_direction == (BLINKER_LEFT if direction == "left" else BLINKER_RIGHT))
          self.held_crossed = self.held_crossing.update(modeldata, direction) or self.held_crossed
          self.held_model_seen = self.held_model_seen or lane_change_prob > 0.5

          # 차선을 넘기 전 BSD 감지 시 차선변경 자동 취소 및 원래 차선 복귀
          # 여기서는 차선변경 요청만 해제하며, 반대 방향의 강제 복귀 조향은 만들지 않습니다.
          before_lane_cross = self._update_bsd_abort_window(modeldata)
          ignore_bsd = (self.laneChangeBsd < 0)
          bsd_detected = torque_override = False
          if not ignore_bsd:
            if self.lane_change_direction == LaneChangeDirection.left:
              bsd_detected = self.left.bsd_hold_counter > 0
              torque_override = carstate.steeringPressed and (carstate.steeringTorque > 0) and (self.laneChangeBsd != 1)
            elif self.lane_change_direction == LaneChangeDirection.right:
              bsd_detected = self.right.bsd_hold_counter > 0
              torque_override = carstate.steeringPressed and (carstate.steeringTorque < 0) and (self.laneChangeBsd != 1)
            else:
              bsd_detected = False
              before_lane_cross = False
              torque_override = False

            if bsd_detected and before_lane_cross and not torque_override:
              self._consume_lane_change_request(carrotMan, "BSD: lane change cancelled before crossing", bsd=True)
              desire_enabled = False

          if self.lane_change_state == LaneChangeState.laneChangeStarting:
            if self.held_repeat_pending:
              if not held_same_direction:
                # None 출력을 낸 뒤 놓았을 때 3/4를 되살리면 새 모델 요청이 되므로 바로 끝냅니다.
                self._consume_lane_change_request(carrotMan, "Held lane change completed",
                                                  preserve_new_stalk=True, carstate=carstate)
                desire_enabled = False
              elif bsd_detected and not ignore_bsd and not torque_override:
                self._consume_lane_change_request(carrotMan, "BSD: next lane blocked", bsd=True)
                desire_enabled = False
              elif self._held_next_lane_ready(carstate, modeldata):
                self.held_repeat_pending = False
                self.lane_change_ll_prob = 1.0
                self.held_crossed = self.held_model_seen = False
                self.held_crossing.begin(modeldata, direction)
                self.lane_change_stalk_counts = (self.prev_left_stalk_count, self.prev_right_stalk_count)
            elif lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
              if held_same_direction:
                # 모델은 rising edge만 받으므로 실제 1회 완료 후 None 한 프레임으로 다음 요청을 준비합니다.
                if self.held_crossed and self.held_model_seen:
                  self.held_repeat_pending = True
              else:
                self.lane_change_state = LaneChangeState.laneChangeFinishing

        elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
          self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)
          if self.lane_change_ll_prob > 0.99:
            self.lane_change_direction = LaneChangeDirection.none
            if self.lane_change_driver_request:
              self._consume_lane_change_request(carrotMan, "Lane change completed",
                                                preserve_new_stalk=True, carstate=carstate)
              desire_enabled = False
            elif desire_enabled:
              self.lane_change_state = LaneChangeState.preLaneChange
              self.next_lane_change = False
            else:
              self.lane_change_state = LaneChangeState.off
    
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

    # 대기 입력이 끝난 뒤 남은 램프만으로 같은 원터치를 다시 시작하지 않습니다.
    if (previous_lane_change_state == LaneChangeState.preLaneChange
        and self.lane_change_state == LaneChangeState.off and self.one_touch_triggered
        and not driver_enabled and not atc_enabled):
      self._consume_lane_change_request(carrotMan, "Lane change request ended")
      self.prev_desire_enabled = False

    # 시작으로 전환된 그 프레임부터 관측합니다. 뒤늦게 정상 차선이 보여도 재무장하지 않습니다.
    if self.lane_change_state == LaneChangeState.laneChangeStarting:
      if previous_lane_change_state != LaneChangeState.laneChangeStarting:
        self._update_bsd_abort_window(modeldata, new_start=True)
        direction = "left" if self.lane_change_direction == LaneChangeDirection.left else "right"
        self.held_crossing.begin(modeldata, direction)
        self.held_crossed = self.held_model_seen = self.held_repeat_pending = False
        # Do not certify a one-touch from a stalk counter edge alone. That
        # edge can provisionally set one_touch_triggered without ever observing
        # physical lever depth 1; a full-lock lever depth 2 is not one-touch.
        physical_one_touch = (
          driver_enabled and self.one_touch_triggered
          and self.stalk_max_level == 1
          and carstate.blinkerLever in (0, 1)
          and not self.one_touch_completed and not self.bsd_stalk_cancelled
          and ((self.stalk_direction == BLINKER_LEFT and self.lane_change_direction == LaneChangeDirection.left)
               or (self.stalk_direction == BLINKER_RIGHT and self.lane_change_direction == LaneChangeDirection.right))
        )
        self.lane_change_driver_request = physical_one_touch
        self.lane_change_stalk_counts = (self.prev_left_stalk_count, self.prev_right_stalk_count)
    else:
      self.bsd_abort_allowed = False
      self.bsd_abort_direction = LaneChangeDirection.none
      self.bsd_abort_distance = None
      self.bsd_abort_timestamp = 0
      self.held_crossing.reset()
      self.held_crossed = self.held_model_seen = self.held_repeat_pending = False
      if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
        self.lane_change_driver_request = False
        self.lane_change_stalk_counts = None

    # final desire
    if self.turn_direction != TurnDirection.none:
      self.desire = TURN_DESIRES[self.turn_direction]
      self.lane_change_direction = self.turn_direction
    else:
      self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    if self.lane_change_state == LaneChangeState.laneChangeStarting and self.held_repeat_pending:
      self.desire = log.Desire.none

    # keep pulse
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none

    self._update_lane_change_blocked(carstate, side, driver_enabled, blinker_state)
    return self.desire
