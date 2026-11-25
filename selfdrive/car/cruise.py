import math
import time
import numpy as np

from cereal import car, log
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.can_cruise_recovery import CanErrorCruiseRecovery
from openpilot.selfdrive.carrot.route_congestion_cruise import RouteCongestionCruiseCap
from opendbc.car.hyundai.values import HyundaiFlags

from opendbc.car import structs
GearShifter = structs.CarState.GearShifter


# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
# V_CRUISE's are in kph
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = 40
V_CRUISE_INITIAL_EXPERIMENTAL_MODE = 105
IMPERIAL_INCREMENT = round(CV.MPH_TO_KPH, 1)  # round here to avoid rounding errors incrementing set speed

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
CRUISE_LONG_PRESS = 50
CRUISE_NEAREST_FUNC = {
  ButtonType.accelCruise: math.ceil,
  ButtonType.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  ButtonType.accelCruise: +1,
  ButtonType.decelCruise: -1,
}


class VCruiseHelper:
  def __init__(self, CP):
    self.CP = CP
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.v_cruise_kph_last = 0
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}

  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def update_v_cruise(self, CS, enabled, is_metric):
    self.v_cruise_kph_last = self.v_cruise_kph

    if CS.cruiseState.available:
      if not self.CP.pcmCruise:
        # if stock cruise is completely disabled, then we can use our own set speed logic
        self._update_v_cruise_non_pcm(CS, enabled, is_metric)
        self.v_cruise_cluster_kph = self.v_cruise_kph
        self.update_button_timers(CS, enabled)
      else:
        self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
        self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        if CS.cruiseState.speed == 0:
          self.v_cruise_kph = V_CRUISE_UNSET
          self.v_cruise_cluster_kph = V_CRUISE_UNSET
        elif CS.cruiseState.speed == -1:
          self.v_cruise_kph = -1
          self.v_cruise_cluster_kph = -1
    else:
      self.v_cruise_kph = V_CRUISE_UNSET
      self.v_cruise_cluster_kph = V_CRUISE_UNSET

  def _update_v_cruise_non_pcm(self, CS, enabled, is_metric):
    # handle button presses. TODO: this should be in state_control, but a decelCruise press
    # would have the effect of both enabling and changing speed is checked after the state transition
    if not enabled:
      return

    long_press = False
    button_type = None

    v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers and not b.pressed:
        if self.button_timers[b.type.raw] > CRUISE_LONG_PRESS:
          return  # end long press
        button_type = b.type.raw
        break
    else:
      for k, timer in self.button_timers.items():
        if timer and timer % CRUISE_LONG_PRESS == 0:
          button_type = k
          long_press = True
          break

    if button_type is None:
      return

    # Don't adjust speed when pressing resume to exit standstill
    cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
    if button_type == ButtonType.accelCruise and cruise_standstill:
      return

    # Don't adjust speed if we've enabled since the button was depressed (some ports enable on rising edge)
    if not self.button_change_states[button_type]["enabled"]:
      return

    v_cruise_delta = v_cruise_delta * (5 if long_press else 1)
    if long_press and self.v_cruise_kph % v_cruise_delta != 0:  # partial interval
      self.v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](self.v_cruise_kph / v_cruise_delta) * v_cruise_delta
    else:
      self.v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    # If set is pressed while overriding, clip cruise speed to minimum of vEgo
    if CS.gasPressed and button_type in (ButtonType.decelCruise, ButtonType.setCruise):
      self.v_cruise_kph = max(self.v_cruise_kph, CS.vEgo * CV.MS_TO_KPH)

    self.v_cruise_kph = np.clip(round(self.v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

  def update_button_timers(self, CS, enabled):
    # increment timer for buttons still pressed
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers:
        # Start/end timer and store current state on change of button pressed
        self.button_timers[b.type.raw] = 1 if b.pressed else 0
        self.button_change_states[b.type.raw] = {"standstill": CS.cruiseState.standstill, "enabled": enabled}

  def initialize_v_cruise(self, CS, experimental_mode: bool) -> None:
    # initializing is handled by the PCM
    if self.CP.pcmCruise:
      return

    initial = V_CRUISE_INITIAL_EXPERIMENTAL_MODE if experimental_mode else V_CRUISE_INITIAL

    if any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents) and self.v_cruise_initialized:
      self.v_cruise_kph = self.v_cruise_kph_last
    else:
      self.v_cruise_kph = int(round(np.clip(CS.vEgo * CV.MS_TO_KPH, initial, V_CRUISE_MAX)))

    self.v_cruise_cluster_kph = self.v_cruise_kph



from openpilot.common.params import Params
#from openpilot.selfdrive.selfdrived.events import Events
#EventName = log.OnroadEvent.EventName

class VCruiseCarrot:
  def __init__(self, CP):
    self.CP = CP
    self.frame = 0
    self.params_memory = Params("/dev/shm/params")
    self.params = Params()
    self.can_cruise_recovery = CanErrorCruiseRecovery()
    self.can_error_cruise_resume = self.params.get_int("CanErrorCruiseResume") > 0
    self.v_cruise_kph = 20 #V_CRUISE_UNSET
    self.v_cruise_cluster_kph = 20 #V_CRUISE_UNSET
    self.v_cruise_kph_last = 20

    self.enabled_last = False

    self.long_pressed = False
    self.button_cnt = 0
    self.button_prev = ButtonType.unknown
    self.button_long_time = 50

    self.is_metric = True

    self.v_ego_kph_set = 0
    self._cruise_speed_min, self._cruise_speed_max = 5, 161
    self._cruise_speed_unit = 10
    self._cruise_speed_unit_basic = 1
    self._cruise_button_mode = 2
    self._cancel_button_mode = 0
    self._lfa_button_mode = 0

    self._gas_pressed_count = 0
    self._gas_pressed_count_last = 0
    self._gas_pressed_value = 0
    self._gas_tok_timer = int(0.4 / 0.01) # 0.4 sec
    self._gas_tok = False
    self._brake_pressed_count = 0
    self._soft_hold_count = 0
    self._soft_hold_active = 0
    self._cruise_ready = False
    self._cruise_cancel_state = False
    self._pause_auto_speed_up = False
    self._activate_cruise = 0
    self._lat_enabled = self.params.get_int("AutoEngage") > 0
    self._v_cruise_kph_at_brake = 0
    self.cruise_state_available_last = False

    self._paddle_decel_active = False
    self.carrot_cruise_active = False
    self.carrot_cruise_min_timer = 0  # <--- [추가] 5초 타이머 변수
    # OpenPilot's requested set speed immediately before Carrot Cruise starts.
    # Independent of the brake/resume backup and the temporary coast setpoint.
    self._carrot_cruise_saved_op_speed = None

    #self.events = []
    self.xState = 0
    self.trafficState = 0
    self.aTarget = 0
    self.nRoadLimitSpeed = 30
    self.desiredSpeed = 250
    self.road_limit_kph = 30
    self.nRoadLimitSpeed_last = 30

    self.carrot_cmd_index_last = 0
    self.carrot_cmd_index = 0
    self.carrot_cmd = ""
    self.carrot_arg = ""


    self._cancel_timer = 0
    self.d_rel = 0
    self.v_rel = 0
    self.v_lead_kph = 0
    self.model_v_kph = 0

    self._log_timer = 0
    self._log_timeout = int(3/0.01)
    self.log = ""

    self.autoCruiseControl = 0
    self.autoCruiseControl_cancel_timer = 0
    self.AutoSpeedUptoRoadSpeedLimit = 0.0

    self.current_driving_mode = self.params.get_int("MyDrivingMode")
    self.route_congestion_cap = RouteCongestionCruiseCap()
    self._auto_mode_manual_speed_change = False
    self._auto_mode_notification_sent = False
    self._auto_mode_automatic_target_change = False
    self.auto_speed_offset = 10.0
    self._auto_mode_active_last = False
    self._reset_auto_mode_session()

    self.useLaneLineSpeed = self.params.get_int("UseLaneLineSpeed")
    self.useLaneLineSpeedApply = self.useLaneLineSpeed


  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def _add_log(self, log):
    if len(log) == 0:
      self._log_timer = max(0, self._log_timer - 1)
      if self._log_timer <= 0:
        self.log = ""
        #self.event = -1
    else:
      self.log = log
      #self.event = event
      self._log_timer = self._log_timeout

  def update_params(self, is_metric):
    unit_factor = 1.0 if is_metric else CV.MPH_TO_KPH
    if self.frame % 10 == 0:
      self.autoCruiseControl = self.params.get_int("AutoCruiseControl") * unit_factor
      self.can_error_cruise_resume = self.params.get_int("CanErrorCruiseResume") > 0
      self.autoGasTokSpeed = self.params.get_int("AutoGasTokSpeed") * unit_factor
      self.autoGasSyncSpeed = self.params.get_int("AutoGasSyncSpeed")
      self.applyModelSpeed = self.params.get_float("ApplyModelSpeed") * 0.01
      self.autoSpeedUptoRoadSpeedLimit = self.params.get_float("AutoSpeedUptoRoadSpeedLimit") * 0.01
      self.autoRoadSpeedAdjust = self.params.get_float("AutoRoadSpeedAdjust") * 0.01

      useLaneLineSpeed = self.params.get_int("UseLaneLineSpeed") * unit_factor
      if self.useLaneLineSpeed != useLaneLineSpeed:
        self.useLaneLineSpeedApply = useLaneLineSpeed
      self.useLaneLineSpeed = useLaneLineSpeed

      self.speed_from_pcm = self.params.get_int("SpeedFromPCM")
      self._cruise_speed_unit = self.params.get_int("CruiseSpeedUnit")
      self._cruise_button_long_delay = self.params.get_int("CruiseButtonLongDelay")
      self._cruise_speed_unit_basic = self.params.get_int("CruiseSpeedUnitBasic")
      self._paddle_mode = self.params.get_int("PaddleMode")
      self._cruise_button_mode = self.params.get_int("CruiseButtonMode")
      self._cancel_button_mode = self.params.get_int("CancelButtonMode")
      self._lfa_button_mode = self.params.get_int("LfaButtonMode")
      self.autoRoadSpeedLimitOffset = self.params.get_int("AutoRoadSpeedLimitOffset")
      self.autoNaviSpeedSafetyFactor = self.params.get_float("AutoNaviSpeedSafetyFactor") * 0.01
      self.cruiseOnDist = self.params.get_float("CruiseOnDist") * 0.01
      cruiseSpeed1 = self.params.get_float("CruiseSpeed1") * unit_factor
      cruiseSpeed2 = self.params.get_float("CruiseSpeed2") * unit_factor
      cruiseSpeed3 = self.params.get_float("CruiseSpeed3") * unit_factor
      cruiseSpeed4 = self.params.get_float("CruiseSpeed4") * unit_factor
      cruiseSpeed5 = self.params.get_float("CruiseSpeed5") * unit_factor
      if cruiseSpeed1 <= 0:
        if self.autoRoadSpeedLimitOffset < 0:
          cruiseSpeed1 = self.nRoadLimitSpeed * self.autoNaviSpeedSafetyFactor
        else:
          cruiseSpeed1 = self.nRoadLimitSpeed + self.autoRoadSpeedLimitOffset
      self._cruise_speed_table = [cruiseSpeed1, cruiseSpeed2, cruiseSpeed3, cruiseSpeed4, cruiseSpeed5]

  def update_v_cruise(self, CS, sm, is_metric):
    self._add_log("")
    self._auto_mode_notification_sent = False
    self._auto_mode_automatic_target_change = False
    self.update_params(is_metric)
    self.frame += 1
    if CS.gearShifter != GearShifter.drive:
      self.autoCruiseControl_cancel_timer = 20 * 100  # 20 sec
    else:
      self.autoCruiseControl_cancel_timer = max(0, self.autoCruiseControl_cancel_timer - 1)

    CC = sm['carControl']
    if sm.alive['carrotMan']:
      carrot_man = sm['carrotMan']
      self.nRoadLimitSpeed = carrot_man.nRoadLimitSpeed
      self.desiredSpeed = carrot_man.desiredSpeed
      self.desiredSource = carrot_man.desiredSource   # <--- 이 줄을 반드시 추가해야 합니다!
      self.carrot_cmd_index = carrot_man.carrotCmdIndex
      self.carrot_cmd = carrot_man.carrotCmd
      self.carrot_arg = carrot_man.carrotArg
    if sm.alive['longitudinalPlan']:
      lp = sm['longitudinalPlan']
      self.xState = lp.xState
      self.trafficState = lp.trafficState
      self.aTarget = lp.aTarget
    if sm.alive['radarState']:
      lead = sm['radarState'].leadOne
      self.d_rel = lead.dRel if lead.status else 0
      self.v_rel = lead.vRel if lead.status else 0
      self.v_lead_kph = lead.vLeadK * CV.MS_TO_KPH if lead.status else 0
    if sm.alive['drivingModelData']:
      self.model_v_kph = sm['drivingModelData'].action.desiredVelocity * CV.MS_TO_KPH

    self.v_cruise_kph_last = self.v_cruise_kph
    self.is_metric = is_metric

    self._cancel_timer = max(0, self._cancel_timer - 1)

    #self.events = []
    self.v_ego_kph_set = int(CS.vEgoCluster * CV.MS_TO_KPH + 0.5)
    self._activate_cruise = 0
    self._prepare_brake_gas(CS, CC)
    if CC.enabled:
      self._cruise_ready = False
      
    # 버튼 입력을 처리해서 v_cruise_kph를 가져오는 기존 코드
    recovery_user_command = (self.carrot_cmd_index != self.carrot_cmd_index_last
                             and self.carrot_cmd in ("CRUISE", "SPEED"))
    v_cruise_kph = self._update_cruise_buttons(CS, CC, self.v_cruise_kph)

    # Naver route congestion is a low-confidence preview, not an obstacle
    # sensor. Only mode 5 with engaged cruise may lower the requested setpoint,
    # after two separate fresh navigation route messages confirm the same jam.
    v_cruise_kph, jam_reduced = self.route_congestion_cap.update(
      float(v_cruise_kph), float(CS.vEgo) * CV.MS_TO_KPH, sm,
      engaged=(self.current_driving_mode == 5 and CC.enabled and CS.cruiseState.available
               and CS.gearShifter == GearShifter.drive and CS.canValid
               and not CS.brakePressed and not CS.regenBraking
               and not self._cruise_cancel_state and self._activate_cruise >= 0
               and (not self.CP.pcmCruise or self.speed_from_pcm != 1)),
      manual_speed_change=self._auto_mode_manual_speed_change,
      gas_pressed=CS.gasPressed,
    )
    if jam_reduced:
      # The navigation-triggered cap is another automatic mode-5 target
      # change. The shared notification below avoids double chimes if a
      # camera/road-limit change happened on the same update.
      self._add_log("Auto congestion cap: 80 km/h")

    # Notify once for every *actual* automatic mode-5 set-speed change:
    # road limit/camera, tollgate entry/exit, and upcoming congestion.
    # Existing mode-5 prompts already issue audio10 through CarrotServ.
    # Emit a direct audio10 only when no prompt was sent on this update.
    # Do not chime for a manual speed choice, a disengaged cruise, or a
    # policy which leaves the requested target unchanged.
    if ((self._auto_mode_automatic_target_change or jam_reduced)
        and not self._auto_mode_notification_sent
        and not self._auto_mode_manual_speed_change
        and self.current_driving_mode == 5 and CC.enabled and CS.cruiseState.available
        and CS.gearShifter == GearShifter.drive and CS.canValid
        and not CS.brakePressed and not CS.regenBraking and not self._cruise_cancel_state
        and self._activate_cruise >= 0):
      if not jam_reduced:
        # The HUD uses this event to blink AUTO even without a text prompt.
        self._add_log(f"Auto target change: {self.v_cruise_kph:g} -> {float(v_cruise_kph):g} km/h")
      self._auto_mode_chime()

    if self._activate_cruise > 0:
      #self.events.append(EventName.buttonEnable)
      self._cruise_ready = False
    elif self._activate_cruise < 0:
      #self.events.append(EventName.buttonCancel)
      self._cruise_ready = True if self._activate_cruise == -2 else False

    # [방어막 개폐] 목표 속도가 정확히 0일 때만 예외적으로 하한선을 0으로 개방
    # 오토모드나 당근크루즈 자동 감속 등 모든 자동 제어는 self._cruise_speed_min(5)에 걸립니다.
    dynamic_min = 0 if v_cruise_kph == 0 else self._cruise_speed_min

    if CS.cruiseState.available:
      if not self.cruise_state_available_last:
        self._lat_enabled = True
        v_cruise_kph = self.v_ego_kph_set
      if not self.CP.pcmCruise:
        # if stock cruise is completely disabled, then we can use our own set speed logic
        self.v_cruise_kph = np.clip(v_cruise_kph, dynamic_min, self._cruise_speed_max)
        self.v_cruise_cluster_kph = self.v_cruise_kph
      else:
        if self.speed_from_pcm == 1:
          self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
          self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        else:
          self.v_cruise_kph = np.clip(v_cruise_kph, 30, self._cruise_speed_max)
          self.v_cruise_cluster_kph = self.v_cruise_kph
    else:
      self.v_cruise_kph = np.clip(v_cruise_kph, dynamic_min, self._cruise_speed_max) 
      self.v_cruise_cluster_kph = self.v_cruise_kph 

    self._update_can_cruise_recovery(CS, CC, sm, driver_command=recovery_user_command)
    self.cruise_state_available_last = CS.cruiseState.available
    self.enabled_last = CC.enabled

  def _update_can_cruise_recovery(self, CS, CC, sm, driver_command=False):
    now_s = time.monotonic()
    events = sm['onroadEvents']
    event_name = log.OnroadEvent.EventName
    transient = (event_name.canError, event_name.commIssue, event_name.commIssueAvgFreq,
                 event_name.locationdTemporaryError)
    events_fresh = (sm.valid['onroadEvents'] and sm.alive['onroadEvents']
                    and 0 <= now_s - sm.recv_time['onroadEvents'] <= 1.5)
    can_fault = (not CS.canValid and not CS.canTimeout and events_fresh
                 and any(e.name == event_name.canError for e in events))
    blocked = (CS.brakePressed or CS.gasPressed or CS.regenBraking or CS.steeringPressed
               or CS.doorOpen or CS.seatbeltUnlatched or CS.canTimeout
               or self._cruise_cancel_state or self.autoCruiseControl_cancel_timer > 0
               or any(e.userDisable or ((e.immediateDisable or e.softDisable) and e.name not in transient)
                      for e in events))
    driver_input = (driver_command or self.button_cnt > 0 or any(b.type != ButtonType.unknown for b in CS.buttonEvents)
                    or self._activate_cruise != 0)
    manual_enable = any(b.type in (ButtonType.accelCruise, ButtonType.decelCruise,
                                   ButtonType.resumeCruise, ButtonType.lfaButton)
                        for b in CS.buttonEvents)
    allowed = (self.can_error_cruise_resume and self.CP.brand == 'hyundai'
               and self.CP.flags & HyundaiFlags.CANFD and self.CP.openpilotLongitudinalControl
               and not self.CP.pcmCruise and not self.CP.passive
               and CS.gearShifter == GearShifter.drive and not CS.standstill
               and math.isfinite(CS.vEgo) and CS.vEgo > 0
               and self._lat_enabled and CS.latEnabled and CC.latActive and CS.cruiseState.available)
    panda_ok = bool(sm['pandaStates']) and all(not p.safetyRxChecksInvalid and not p.faults and not p.heartbeatLost
                                             for p in sm['pandaStates'])
    # onroadEvents is event-driven: a legitimate burst must not fail a 1 Hz frequency check.
    critical = ('selfdriveState', 'pandaStates', 'carControl', 'longitudinalPlan', 'radarState', 'modelV2')
    fresh = all(0 <= now_s - sm.recv_time[name] <= (0.25 if name == 'pandaStates' else 0.15)
                and (self.can_cruise_recovery.pending_since is None
                     or sm.recv_time[name] >= self.can_cruise_recovery.pending_since)
                for name in critical)
    ready = (CS.canValid and not CS.canTimeout and sm.all_checks(critical) and events_fresh and fresh
             and sm['selfdriveState'].engageable and not any(e.noEntry for e in events) and panda_ok)
    previous_status = self.can_cruise_recovery.status
    target = self.can_cruise_recovery.update(
      now_s=now_s, enabled=CC.enabled, set_speed_kph=self.v_cruise_kph,
      can_fault=can_fault, blocked=blocked, ready=ready, allowed=bool(allowed),
      driver_input=driver_input, manual_enable=manual_enable,
    )
    if target is not None:
      # This is a request through carSpecific/StateMachine, never a forced enabled state or a fake + button.
      self._activate_cruise = 1
      self._cruise_ready = False
      self.v_cruise_kph = self.v_cruise_cluster_kph = target
      self._add_log(f"CAN recovery requested: {target:.0f} km/h")
    elif self.can_cruise_recovery.status != previous_status:
      self._add_log(f"CAN recovery: {self.can_cruise_recovery.status}")

  def initialize_v_cruise(self, CS, experimental_mode: bool) -> None:
    return
    # initializing is handled by the PCM
    if self.CP.pcmCruise and self.speed_from_pcm == 1:
      return

    initial = V_CRUISE_INITIAL_EXPERIMENTAL_MODE if experimental_mode else CS.vEgoCluster * CV.MS_TO_KPH

    v_ego_kph = int(round(np.clip(CS.vEgoCluster * CV.MS_TO_KPH, initial, V_CRUISE_MAX)))
    print(CS.buttonEvents)
    if any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents): # and self.v_cruise_initialized:
      self.v_cruise_kph = max(self._v_cruise_kph_at_brake, v_ego_kph) if self._v_cruise_kph_at_brake > 0 else self.v_cruise_kph_last
      self._add_log(f"{self.v_cruise_kph},{self._v_cruise_kph_at_brake} Cruise resume")
    else:
      self.v_cruise_kph = v_ego_kph
      self._add_log(f"{self.v_cruise_kph} Cruise Set")

    self.v_cruise_kph = np.clip(self.v_cruise_kph, self._cruise_speed_min, self._cruise_speed_max)
    self.v_cruise_cluster_kph = self.v_cruise_kph

  def _prepare_buttons(self, CS, v_cruise_kph):
    button_kph = v_cruise_kph
    button_type = 0
    buttonEvents = CS.buttonEvents

    SPEED_UP_UNIT = self._cruise_speed_unit_basic
    SPEED_DOWN_UNIT = self._cruise_speed_unit if self._cruise_button_mode in [1, 2, 3] else self._cruise_speed_unit_basic
    V_CRUISE_DELTA = 30  #크루즈버튼 롱클릭 변경 단위
    is_metric = self.is_metric

    # long press tracking
    if self.button_cnt > 0:
      self.button_cnt += 1

    for b in buttonEvents:
      bt = b.type

      # ▼▼▼ [삭제] 패들 즉시 반응 로직 6줄 삭제 (또는 주석 처리) ▼▼▼
      # if bt in [ButtonType.paddleLeft, ButtonType.paddleRight] and b.pressed:
      #   # Paddle: 즉시 이벤트 발생
      #   button_type = bt
      #   self.long_pressed = False
      #   self.button_cnt = 0
      #   continue
      # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

      # ▼▼▼ [수정] 아래 리스트에 패들 버튼 2개를 추가해 줍니다 ▼▼▼
      if b.pressed and self.button_cnt == 0 and bt in [
        ButtonType.accelCruise, ButtonType.decelCruise,
        ButtonType.gapAdjustCruise, ButtonType.cancel,
        ButtonType.lfaButton,
        ButtonType.paddleLeft, ButtonType.paddleRight  # <--- 추가됨!
      ]:

        self.button_cnt = 1
        self.button_prev = bt
        self.button_long_time = self._cruise_button_long_delay if bt in [ButtonType.accelCruise, ButtonType.decelCruise] else self._cruise_button_long_delay + 30

      elif not b.pressed and self.button_cnt > 0 and bt == self.button_prev:
        if bt == ButtonType.cancel:
          button_type = bt
        elif not self.long_pressed:          
          if bt == ButtonType.accelCruise:
            unit = SPEED_UP_UNIT if is_metric else SPEED_UP_UNIT * CV.MPH_TO_KPH
            button_kph = math.ceil((button_kph + 0.01) / unit) * unit
          elif bt == ButtonType.decelCruise:
            unit = SPEED_DOWN_UNIT if is_metric else SPEED_DOWN_UNIT * CV.MPH_TO_KPH
            new_kph = math.floor((button_kph - 0.01) / unit) * unit
            # [수정] 짧게 누를 땐 평소처럼 최소 5km/h 방어 (이미 0이라면 0 유지)
            button_kph = max(self._cruise_speed_min, new_kph) if button_kph > 0 else 0
          button_type = bt
        self.long_pressed = False
        self.button_cnt = 0

    # Long press 처리
    if self.button_cnt > self.button_long_time:
      self.long_pressed = True
      bt = self.button_prev

      if bt in [ButtonType.accelCruise, ButtonType.decelCruise]:
        # 1초(100프레임) 주기마다 계속해서 속도가 조절되도록 변경
        if (self.button_cnt - self.button_long_time) % 100 == 1:
          if bt == ButtonType.accelCruise:
            button_kph += 30
          else:
            new_kph = button_kph - 30
            # [수정] 일반 롱프레스 시에도 무조건 최소 5km/h 방어
            button_kph = max(self._cruise_speed_min, new_kph) if button_kph > 0 else 0
          button_type = bt
          
        # [우회 핵심] 딱 3초(300프레임) 입력되는 순간에만 방어막 뚫고 0 세팅!
        if self.button_cnt == 300 and bt == ButtonType.decelCruise:
          button_kph = 0
          button_type = bt
          
      else: 
        if self.button_cnt == self.button_long_time + 1:
          button_type = bt

    return button_kph, button_type, self.long_pressed

  def _carrot_command(self, v_cruise_kph, button_type, long_pressed):
    if self.carrot_cmd_index_last != self.carrot_cmd_index:
      self.carrot_cmd_index_last = self.carrot_cmd_index
      print(f"Carrot command(cruise.py): {self.carrot_cmd} {self.carrot_arg}")
      if self.carrot_cmd == "CRUISE":
        if self.carrot_arg == "OFF":
          self._cruise_control(-2, -1, "Cruise off (carrot command)")
        elif self.carrot_arg == "ON":
          self._cruise_control(1, -1, "Cruise on (carrot command)")
        elif self.carrot_arg == "GO":
          if button_type == 0:
            button_type = ButtonType.accelCruise
            long_pressed = False
            self._add_log("Cruise accelCruise (carrot command)")
        elif self.carrot_arg == "STOP":
          v_cruise_kph = 5
          self._add_log("Cruise stop (carrot command)")

      elif self.carrot_cmd == "SPEED":
        if self.carrot_arg == "UP":
          v_cruise_kph = v_cruise_kph + 5 if self.current_driving_mode == 5 else self._auto_speed_up(v_cruise_kph)
          self._add_log("Cruise speed up (carrot command)")
        elif self.carrot_arg == "DOWN":
          if v_cruise_kph > 20: #이부분 20에서 10으로 수정
            v_cruise_kph -= 10
            self._add_log("Cruise speed downup (carrot command)")
        else:
          speed_kph = int(self.carrot_arg)
          if 0 < speed_kph < 200:
            v_cruise_kph = speed_kph
            self._add_log(f"Cruise speed set to {v_cruise_kph} (carrot command)")       
    
    return v_cruise_kph, button_type, long_pressed

  def _update_cruise_buttons(self, CS, CC, v_cruise_kph):
    if self.frame % 10 == 0:
      self.current_driving_mode = self.params.get_int("MyDrivingMode")
    button_kph, button_type, long_pressed = self._prepare_buttons(CS, v_cruise_kph)

    # Preserve the physical input before remote commands and restore actions
    # translate buttons. Only a real +/- action may teach the auto offset.
    physical_speed_button = button_type in (ButtonType.accelCruise, ButtonType.decelCruise)
    remote_speed_command = (self.carrot_cmd_index != self.carrot_cmd_index_last
                            and (self.carrot_cmd == "SPEED" or (self.carrot_cmd == "CRUISE" and self.carrot_arg == "GO")))
    auto_restore_speed = None
    automatic_carrot_restore_target = None
    lfa_carrot_restored = False

    v_cruise_kph, button_type, long_pressed = self._carrot_command(v_cruise_kph, button_type, long_pressed)
    
    self.button_type_pressed = button_type  # ▼▼▼ [핵심 추가] 오토모드 오프셋 조절을 위해 눌린 버튼 기억 ▼▼▼

    # ▼▼▼ [추가] 당근크루즈 상태에서 LFA 버튼을 누르면 '+' 버튼처럼 작동하게 치환 ▼▼▼
    if not long_pressed and button_type == ButtonType.lfaButton and getattr(self, '_lfa_button_mode', 0) == 2:
      if getattr(self, 'carrot_cruise_active', False):
        saved_speed = self._carrot_cruise_saved_op_speed
        if saved_speed is not None:
          auto_restore_speed = v_cruise_kph
          automatic_carrot_restore_target = saved_speed
          v_cruise_kph = max(v_cruise_kph, saved_speed)
          button_kph = v_cruise_kph
        self._carrot_cruise_saved_op_speed = None
        self._v_cruise_kph_at_brake = 0
        self.carrot_cruise_active = False
        lfa_carrot_restored = True
        button_type = ButtonType.accelCruise
        self._add_log("Carrot Cruise OFF & Restore Speed (LFA toggle)")
    # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

    if button_type in [ButtonType.accelCruise, ButtonType.decelCruise]:
      self._paddle_decel_active = False
      if self.autoCruiseControl_cancel_timer > 0:
        self._add_log(f"AutoCruiseControl cancel timer RESET {button_type}")
        self.autoCruiseControl_cancel_timer = 0
      if self._cruise_cancel_state:
        self._add_log(f"Cruise Cancel state RESET {button_type}")
        self._cruise_cancel_state = False

    if not long_pressed:
      if button_type == ButtonType.accelCruise:
        self._lat_enabled = True
        self._pause_auto_speed_up = False
        if lfa_carrot_restored:
          # LFA has already restored the target; it is not an extra '+'.
          self._soft_hold_active = 0
        elif self.carrot_cruise_active:
          self._soft_hold_active = 0
          saved_speed = self._carrot_cruise_saved_op_speed
          if saved_speed is not None:
            auto_restore_speed = v_cruise_kph
            automatic_carrot_restore_target = saved_speed
            v_cruise_kph = max(v_cruise_kph, saved_speed)
          # Never fall back to a possibly stale brake backup in Carrot Cruise.
          self._carrot_cruise_saved_op_speed = None
          self._v_cruise_kph_at_brake = 0
          self.carrot_cruise_active = False
        elif self._soft_hold_active > 0:
          self._soft_hold_active = 0
        elif self._v_cruise_kph_at_brake > 0 and v_cruise_kph < self._v_cruise_kph_at_brake:
          auto_restore_speed = v_cruise_kph
          v_cruise_kph = self._v_cruise_kph_at_brake
          self._v_cruise_kph_at_brake = 0
          self.carrot_cruise_active = False  
        elif self._cruise_ready or not CC.enabled or CS.cruiseState.standstill or self.carrot_cruise_active:
          
          # 최초 작동 시 제한속도 +10 로직
          if not CC.enabled and getattr(self, 'is_first_activation', True):
            self.is_first_activation = False  
            if self.nRoadLimitSpeed > 0:
              v_cruise_kph = self.nRoadLimitSpeed + 10  
            else:
              v_cruise_kph = 50  
          
          if False:
            road_limit_kph = self.nRoadLimitSpeed * self.autoSpeedUptoRoadSpeedLimit
            if road_limit_kph > 1.0:
              v_cruise_kph = max(v_cruise_kph, road_limit_kph)

        elif self._cruise_button_mode in [0, 1]:
          v_cruise_kph = button_kph
        else:
          # ▼▼▼ [수정] v_cruise_kph 대신 증가된 속도인 button_kph를 넣어야 정확히 올라갑니다! ▼▼▼
          v_cruise_kph = self._v_cruise_desired(CS, button_kph)
        self.carrot_cruise_active = False

      elif button_type == ButtonType.decelCruise:
        self._lat_enabled = True
        self._pause_auto_speed_up = True

        if self.carrot_cruise_active:
          self.carrot_cruise_active = False  
          v_cruise_kph = max(self.v_ego_kph_set, self._cruise_speed_min)  
          self._add_log("Carrot Cruise OFF & Set to current speed")
        else:
          if self._soft_hold_active > 0:
            self._cruise_control(-1, -1, "Cruise off,softhold mode (decelCruise)")
          elif self._cruise_ready:
            self._paddle_decel_active = True
            pass
          elif not CC.enabled:
            v_cruise_kph = max(self.v_ego_kph_set, self._cruise_speed_min)
            self.is_first_activation = False  
          elif self.v_ego_kph_set > v_cruise_kph + 2 and self._cruise_button_mode in [2, 3]:
            v_cruise_kph = max(self.v_ego_kph_set, self._cruise_speed_min)
          elif self._cruise_button_mode in [0, 1]:
            v_cruise_kph = button_kph
          elif self.v_ego_kph_set < 1.0:
            self.carrot_cruise_active = True
          elif self.v_ego_kph_set > self._cruise_speed_min and v_cruise_kph > self.v_ego_kph_set:
            v_cruise_kph = self.v_ego_kph_set
          else:
            self.carrot_cruise_active = True
            
        self._v_cruise_kph_at_brake = 0

      elif button_type == ButtonType.gapAdjustCruise:
        longitudinalPersonalityMax = self.params.get_int("LongitudinalPersonalityMax")
        if CS.pcmCruiseGap == 0:
          personality = (self.params.get_int('LongitudinalPersonality') - 1) % longitudinalPersonalityMax
        else:
          personality = np.clip(CS.pcmCruiseGap - 1, 0, longitudinalPersonalityMax)
        self.params.put_int_nonblocking('LongitudinalPersonality', personality)
      elif button_type == ButtonType.lfaButton:
        if self._lfa_button_mode == 0:
          self._lat_enabled = not self._lat_enabled
          self._add_log("Lateral " + "enabled" if self._lat_enabled else "disabled")
        elif self._lfa_button_mode == 2:
          self.carrot_cruise_active = True
        else:
          if False: 
            self._paddle_decel_active = False
          else:          
            self._paddle_decel_active = True
      elif button_type == ButtonType.cancel:
        self._paddle_decel_active = False
        if self._cancel_button_mode in [1]:
          self._lat_enabled = False
          self._add_log("Lateral " + "enabled" if self._lat_enabled else "disabled")
        self._cruise_cancel_state = True
        self.carrot_cruise_active = False  
    else:
      if button_type == ButtonType.accelCruise:
        v_cruise_kph = button_kph
        self._v_cruise_kph_at_brake = 0
      elif button_type == ButtonType.decelCruise:
        self._pause_auto_speed_up = True
        v_cruise_kph = button_kph
        self._v_cruise_kph_at_brake = 0
        if v_cruise_kph == 0:
          self.carrot_cruise_active = False # [수정] 0일 땐 타력주행 방어 로직과 충돌 차단
      elif button_type == ButtonType.gapAdjustCruise:
        current_mode = self.params.get_int("MyDrivingMode")
        new_mode = 5 if current_mode == 3 else 3
        self.params.put_int_nonblocking("MyDrivingMode", new_mode)
        self.current_driving_mode = new_mode
        if current_mode == 5 and new_mode == 3:
          # This callback runs only once at the long-press threshold.
          # A dedicated event plays tici_disengaged.wav, never the ordinary
          # auto speed-change bell; no new navigation text is displayed.
          self._auto_mode_off_chime()

      elif button_type == ButtonType.lfaButton:
        useLaneLineSpeed = max(1, self.useLaneLineSpeed)
        self.useLaneLineSpeedApply = useLaneLineSpeed if self.useLaneLineSpeedApply == 0 else 0

      elif button_type == ButtonType.cancel:
        self._cruise_cancel_state = True
        self._lat_enabled = False
        self._paddle_decel_active = False
        self.carrot_cruise_active = False  
        self._add_log("Lateral " + "enabled" if self._lat_enabled else "disabled")

    if self._paddle_mode > 0 and button_type in [ButtonType.paddleLeft, ButtonType.paddleRight]:
      if long_pressed:
        self._paddle_decel_active = False
      else:
        if self._paddle_mode == 3:
          self.carrot_cruise_active = True
        else:
          self._cruise_control(-2, -1, "Cruise off & Ready (paddle)")
          if self._paddle_mode == 2:
            self._paddle_decel_active = True

    elif self._paddle_decel_active:
      if not CC.enabled:
        self._cruise_control(1, -1, "Cruise on (paddle decel)")

    if not hasattr(self, 'prev_carrot_active'):
      self.prev_carrot_active = False

    # Braking cancels Carrot Cruise, but must not replace the saved OpenPilot
    # target with the zero vehicle/coast speed. This only restores the stored
    # setpoint; it never requests cruise engagement or vehicle acceleration.
    if not self.carrot_cruise_active and self.prev_carrot_active:
      if CS.brakePressed and button_type == 0 and self._carrot_cruise_saved_op_speed is not None:
        v_cruise_kph = self._carrot_cruise_saved_op_speed
      self._carrot_cruise_saved_op_speed = None

    if getattr(self, 'carrot_cruise_active', False) and not self.prev_carrot_active:
      # Capture the previous OpenPilot v_cruise, not cluster SCC speed or vEgo.
      # An unset/zero setpoint must never be replaced with a guessed speed.
      previous_op_speed = float(self.v_cruise_kph)
      self._carrot_cruise_saved_op_speed = (previous_op_speed if
        math.isfinite(previous_op_speed) and 0 < previous_op_speed <= self._cruise_speed_max
        and previous_op_speed != V_CRUISE_UNSET else None)
      self.carrot_cruise_min_timer = 500  # <--- [추가] 당근크루즈 켜지면 5초(500프레임) 장전

    # ==============================================================
    # ▼ [통합 복귀 로직] 가속 페달, 정지차량 5초, 목표속도 5초 도달 시 
    # 당근크루즈 자동 해제 및 기존 속도(오토모드 완벽 동기화) 복귀
    # ==============================================================
    # ==============================================================
    if getattr(self, 'carrot_cruise_active', False):
      # ▼▼▼ [추가] 5초 타이머 차감 ▼▼▼
      if self.carrot_cruise_min_timer > 0:
        self.carrot_cruise_min_timer -= 1

      restore_triggered = False
      restore_reason = ""
      
      if not hasattr(self, 'stationary_lead_timer'): self.stationary_lead_timer = 0
      if not hasattr(self, 'target_speed_reach_timer'): self.target_speed_reach_timer = 0
      
      # 1. 가속 페달 개입
      if CS.gasPressed:
        restore_triggered = True
        restore_reason = "Gas Override"
        
      # 2. 전방 정지차량 5초 연속 감지
      elif self.d_rel > 0 and self.v_lead_kph < 3:
        self.stationary_lead_timer += 1
        if self.stationary_lead_timer >= 500:  # 5초
          restore_triggered = True
          restore_reason = "Stationary 5s"
      else:
        self.stationary_lead_timer = 0
        
      # 3. 오직 'VTRUN' (비전 곡률 감속) 목표 속도 도달 시 즉시 복귀 (1초 대기 없음)
      if not restore_triggered:
        current_source = getattr(self, 'desiredSource', '')
        is_vturn_decel = (current_source == "VTRUN")
        
        if is_vturn_decel and self.desiredSpeed < 70 and self.v_ego_kph_set <= self.desiredSpeed + 1 and self.v_ego_kph_set <= 60:
          restore_triggered = True
          restore_reason = f"VTRUN Speed {self.desiredSpeed} Reached"

      # 🚀 복귀 트리거 발동
      if restore_triggered:
        # ▼▼▼ [수정] 가속 페달(Gas Override) 개입 시 5초 타이머 무시하고 즉시 해제! ▼▼▼
        if self.carrot_cruise_min_timer > 0 and restore_reason != "Gas Override":
          restore_triggered = False
        else:
          self.carrot_cruise_active = False
          self.stationary_lead_timer = 0
          self.target_speed_reach_timer = 0
        
          # Restore the OpenPilot setpoint captured before entering Carrot
          # Cruise. Never source it from the live (possibly zero) coast speed.
          saved_speed = self._carrot_cruise_saved_op_speed
          if saved_speed is not None:
            auto_restore_speed = v_cruise_kph
            automatic_carrot_restore_target = saved_speed
            v_cruise_kph = max(v_cruise_kph, saved_speed)
          self._carrot_cruise_saved_op_speed = None
          self._v_cruise_kph_at_brake = 0
            
          self._add_log(f"Carrot Cruise OFF & Restore ({restore_reason})")
    else:
      self.stationary_lead_timer = 0
      self.target_speed_reach_timer = 0
    # ==============================================================

    # Record the FINAL cruise state (after automatic exit), otherwise a
    # re-entry on the next update can miss its new backup capture.
    self.prev_carrot_active = self.carrot_cruise_active

    # Apply mode 5 after the ordinary button/pedal handling so learning uses
    # the actual selected speed, and legacy auto-speed calculations cannot
    # overwrite camera/toll decisions afterwards.
    before_state_speed = v_cruise_kph
    v_cruise_kph = self._update_cruise_state(CS, CC, v_cruise_kph)
    if self.current_driving_mode == 5:
      pedal_speed_change = CS.gasPressed and v_cruise_kph != before_state_speed
      self._auto_mode_manual_speed_change = physical_speed_button or remote_speed_command or pedal_speed_change
      automatic_target = self._update_auto_mode(
        CS, CC, v_cruise_kph, physical_speed_button=physical_speed_button,
        manual_speed_change=self._auto_mode_manual_speed_change,
        restore_speed=auto_restore_speed,
        automatic_carrot_restore_target=automatic_carrot_restore_target,
      )
      # Compare the policy output with the speed after driver/button and
      # ordinary cruise-state handling. Do not treat those manual changes as
      # automatic mode-5 decisions.
      self._auto_mode_automatic_target_change = (
        not self._auto_mode_manual_speed_change and math.isfinite(automatic_target)
        and math.isfinite(v_cruise_kph) and abs(automatic_target - v_cruise_kph) >= 0.5
      )
      return automatic_target
    self._auto_mode_manual_speed_change = False
    self._auto_mode_active_last = False
    self._reset_auto_mode_session()
    return self._auto_speed_up(v_cruise_kph)

  def _reset_auto_mode_session(self):
    self.auto_prev_limit = 0
    self._carrot_auto_shadow_target = None
    self.auto_source_high = None
    self.auto_mode_needs_sync = True
    self.auto_sync_speed = None
    self.auto_baseline_pending = False
    self.auto_can_resume = False
    self.camera_pending = False
    self.auto_camera_override = False
    self.auto_engage_timer = 0
    self.memorized_nav_limit = 0
    self.nav_limit_timer = 0
    self.prev_is_tollgate = False
    self.v_cruise_before_tollgate = 0
    self.expected_tollgate_speed = 0
    self.tollgate_speed_changed = False
    self.tollgate_nav_limit = 0

  @staticmethod
  def _auto_road_limit(value):
    return float(value) if math.isfinite(value) and 30 <= value <= 160 else 0.0

  def _auto_mode_prompt(self, text):
    # CarrotServ consumes the text and emits one audio10 chime.
    # Mark this cycle before the direct-chime fallback is considered.
    try:
      with open("/dev/shm/carrot_prompt_text", "w") as f:
        f.write(text)
      self._auto_mode_notification_sent = True
    except OSError:
      pass

  def _auto_mode_chime(self):
    # Reuse the existing CarrotPlanner audio10 event, without adding
    # navigation text or another audio source.
    try:
      with open("/dev/shm/carrot_prompt", "w"):
        pass
      self._auto_mode_notification_sent = True
    except OSError:
      pass

  @staticmethod
  def _auto_mode_off_chime():
    # CarrotPlanner translates this one-shot marker to EventName.audio0,
    # whose existing soundd mapping plays tici_disengaged.wav once.
    try:
      with open("/dev/shm/carrot_auto_mode_off", "w"):
        pass
    except OSError:
      pass

  def _update_auto_mode(self, CS, CC, v_cruise_kph, *, physical_speed_button,
                        manual_speed_change, restore_speed, automatic_carrot_restore_target=None):
    # The real setpoint is deliberately held at/below vehicle speed in Carrot
    # Cruise. Separately advance the regular mode-5 setpoint so road-limit
    # changes during the coast are not lost when it automatically disengages.
    shadow_target = getattr(self, "_carrot_auto_shadow_target", None)
    if self.carrot_cruise_active:
      if shadow_target is None:
        saved = self._carrot_cruise_saved_op_speed
        shadow_target = saved if saved is not None else v_cruise_kph
      if manual_speed_change:
        # Explicit driver/remote speed choices take precedence over a shadow.
        shadow_target = v_cruise_kph
      updated_target = self._update_auto_mode_target(
        CS, CC, shadow_target, physical_speed_button=physical_speed_button,
        manual_speed_change=manual_speed_change, restore_speed=None,
        automatic_carrot_restore_target=None,
      )
      if self._auto_mode_active_last:
        self._carrot_auto_shadow_target = updated_target
      return min(v_cruise_kph, updated_target)

    if automatic_carrot_restore_target is not None:
      # A zero coast setpoint at standstill is not a valid camera/safety speed
      # cap; do not retain zero as auto_sync_speed after Carrot Cruise ends.
      # Independent obstacle and navigation speed limits remain in effect.
      if restore_speed is not None and restore_speed <= 0:
        restore_speed = None
      # The last mode-5 target is authoritative on an automatic return,
      # including verified road-limit changes during the coast.
      if shadow_target is not None:
        v_cruise_kph = shadow_target
      # The lower coasting speed must not cap a normal automatic restore.
      # Keep the original safety hold path for camera/section/toll states.
      section_active = bool(getattr(CS, "navSectionValid", False)
                            and getattr(CS, "navSectionActive", False))
      toll_active = (getattr(CS, "navTollExist", 0) == 1
                     and getattr(CS, "navLinkClass", 0) == 1)
      camera_active = getattr(CS, "mapSource", 0) == 2
      nav_valid = bool(CS.canValid and getattr(CS, "navDataValid", False))
      if ((nav_valid and not (camera_active or section_active or toll_active or self.prev_is_tollgate))
          or (not nav_valid and not self.camera_pending)):
        restore_speed = None

    self._carrot_auto_shadow_target = None
    return self._update_auto_mode_target(
      CS, CC, v_cruise_kph, physical_speed_button=physical_speed_button,
      manual_speed_change=manual_speed_change, restore_speed=restore_speed,
      automatic_carrot_restore_target=automatic_carrot_restore_target,
    )

  def _update_auto_mode_target(self, CS, CC, v_cruise_kph, *, physical_speed_button,
                               manual_speed_change, restore_speed, automatic_carrot_restore_target=None):
    cancel_held = ((self.button_prev == ButtonType.cancel and self.button_cnt > 0)
                   or any(b.type == ButtonType.cancel and b.pressed for b in CS.buttonEvents))
    if (not CC.enabled or CS.gearShifter != GearShifter.drive or not CS.cruiseState.available
        or CS.brakePressed or CS.regenBraking or cancel_held or self._cruise_cancel_state or self._activate_cruise < 0):
      self._auto_mode_active_last = False
      self._reset_auto_mode_session()
      return v_cruise_kph

    if not self._auto_mode_active_last or not self.enabled_last:
      self._reset_auto_mode_session()
      self._auto_mode_active_last = True
      self.auto_engage_timer = 150 if not self.enabled_last else 0
      # CAN recovery has already chosen the prior trusted set speed. Keep
      # that contract; a later road-limit change can resume normal tracking.
      self.auto_can_resume = self.can_cruise_recovery.status == "requested"
    elif self.auto_engage_timer > 0:
      self.auto_engage_timer -= 1

    if restore_speed is not None:
      self.auto_mode_needs_sync = True
      self.auto_sync_speed = restore_speed

    # Physical input remains authoritative even when navigation is unknown.
    if manual_speed_change and restore_speed is None and (automatic_carrot_restore_target is None or physical_speed_button):
      self.auto_mode_needs_sync = False
      self.auto_sync_speed = None
      self.auto_baseline_pending = True
    if manual_speed_change and self.prev_is_tollgate:
      self.tollgate_speed_changed = True
    if physical_speed_button and CS.mapSource == 2:
      self.auto_camera_override = True
    self.nav_limit_timer = max(0, self.nav_limit_timer - 1)

    # canValid alone does not validate optional cached navigation messages.
    # Unknown navigation must neither change speed nor consume a road event.
    if not CS.canValid or not getattr(CS, 'navDataValid', False):
      if automatic_carrot_restore_target is not None and not self.camera_pending:
        # Unknown navigation must not keep the pre-coast speed as an
        # artificial cap when returning to a driver's existing set speed.
        # This changes the requested setpoint only; normal ACC/lead/curve
        # speed planning still applies independently.
        self.auto_sync_speed = None
        self.auto_mode_needs_sync = False
        self.auto_baseline_pending = True
      self.camera_pending = True
      if not getattr(CS, 'navDataValid', False):
        self.nav_limit_timer = 0
        self.memorized_nav_limit = 0
      return min(v_cruise_kph, self.auto_sync_speed) if self.auto_sync_speed is not None else v_cruise_kph

    map_source = CS.mapSource
    if map_source != 2:
      self.auto_camera_override = False
    elif physical_speed_button:
      self.auto_camera_override = True
    section_active = getattr(CS, 'navSectionValid', False) and getattr(CS, 'navSectionActive', False)
    self.camera_pending = map_source == 2 and not (section_active or self.auto_camera_override)

    speed_limit_pure = self._auto_road_limit(CS.navSpeedLimit)
    # Remember only received valid values. A zero stream must not keep an
    # obsolete nonzero limit alive for an entire camera announcement.
    if speed_limit_pure > 0:
      self.memorized_nav_limit = speed_limit_pure
      self.nav_limit_timer = 100
    elif self.nav_limit_timer > 0:
      speed_limit_pure = self.memorized_nav_limit

    is_tollgate = CS.navTollExist == 1 and CS.navLinkClass == 1
    if CS.navLinkClass == 1:
      self.auto_source_high = True
    elif CS.navLinkClass in (2, 3):
      self.auto_source_high = False
    elif self.auto_source_high is None or manual_speed_change:
      self.auto_source_high = v_cruise_kph >= 90
    # A mainline stays on 4A3 even in congestion or after an auto reduction.
    # For unknown road classes, retain the driver's reference rather than
    # reclassifying the road from this function's own output. A right lane
    # change alone is not proof that the vehicle has taken an exit.
    use_pure = not is_tollgate and CS.navLinkClass not in (2, 3) and (
      CS.navLinkClass == 1 or self.auto_source_high
    )
    target_raw_limit = (speed_limit_pure if use_pure else
                        self._auto_road_limit(CS.speedLimit) if getattr(CS, 'speedLimitDataValid', False) else 0.0)

    # Automatic returns also run the regular mode-5 road-limit calculation
    # on the shadow target; never bypass a limit change on the exit frame.

    if self.auto_can_resume:
      self.auto_mode_needs_sync = False
      self.auto_baseline_pending = True
      self.auto_can_resume = False
      if is_tollgate:
        # A restart inside the toll is not another physical entry. The old
        # session's pre-toll backup is unknown; never guess a restore speed.
        self.prev_is_tollgate = True
        self.v_cruise_before_tollgate = self.expected_tollgate_speed = v_cruise_kph
        self.tollgate_speed_changed = True
    if self.auto_baseline_pending and target_raw_limit > 0:
      self.auto_prev_limit = target_raw_limit
      self.auto_baseline_pending = False

    if manual_speed_change and (automatic_carrot_restore_target is None or physical_speed_button):
      if (physical_speed_button and restore_speed is None and self.auto_engage_timer == 0
          and not self.carrot_cruise_active and target_raw_limit > 0):
        self.auto_speed_offset = float(np.clip(v_cruise_kph - target_raw_limit, 0.0, 10.0))
        self._add_log(f"Auto Offset Saved: +{self.auto_speed_offset}")
      if restore_speed is None:
        # An explicit speed selection also cancels a toll entry on this frame.
        if is_tollgate and not self.prev_is_tollgate:
          self.v_cruise_before_tollgate = v_cruise_kph
          self.expected_tollgate_speed = v_cruise_kph
          self.tollgate_speed_changed = True
          self.tollgate_nav_limit = speed_limit_pure
        self.prev_is_tollgate = is_tollgate
        return v_cruise_kph

    if is_tollgate and not self.prev_is_tollgate:
      self.v_cruise_before_tollgate = v_cruise_kph
      self.tollgate_speed_changed = False
      reduced = math.floor(v_cruise_kph * 0.8 / 5.0 + 0.5) * 5.0
      v_cruise_kph = min(v_cruise_kph, max(30.0, reduced), 110.0)
      self.expected_tollgate_speed = v_cruise_kph
      self.tollgate_nav_limit = speed_limit_pure
      self.prev_is_tollgate = True
      self._add_log(f"Tollgate IN: Speed -20% ({v_cruise_kph}km/h)")
      self._auto_mode_prompt(f"톨게이트 진입: {int(v_cruise_kph)}km/h 🔔")
      return v_cruise_kph
    if is_tollgate:
      # Neither ordinary limit updates nor synthetic restores may undo the
      # toll reduction. Only an actual driver speed choice cancels its hold.
      return v_cruise_kph if self.tollgate_speed_changed else min(v_cruise_kph, self.expected_tollgate_speed)
    if self.prev_is_tollgate:
      deferred_restore = self.auto_sync_speed is not None
      road_changed = (not self.tollgate_speed_changed and target_raw_limit > 0
                      and self.tollgate_nav_limit > 0 and target_raw_limit != self.tollgate_nav_limit)
      if road_changed:
        # A genuinely changed road limit takes precedence over an old backup.
        self.auto_mode_needs_sync = True
        if self.auto_sync_speed is None:
          self.auto_sync_speed = v_cruise_kph
      elif not self.tollgate_speed_changed:
        v_cruise_kph = min(110.0, max(v_cruise_kph, self.v_cruise_before_tollgate))
        # During a coast the wrapper caps only the real setpoint. Do not
        # apply the coast clamp to this shadow auto-mode target.
        self._add_log(f"Tollgate OUT: Restore Speed ({v_cruise_kph}km/h)")
      self.prev_is_tollgate = False
      self.v_cruise_before_tollgate = self.expected_tollgate_speed = 0
      self.tollgate_nav_limit = 0
      self.tollgate_speed_changed = False
      if not deferred_restore and not road_changed:
        self.auto_mode_needs_sync = False
        self.auto_baseline_pending = target_raw_limit <= 0
        if target_raw_limit > 0:
          self.auto_prev_limit = target_raw_limit
        return v_cruise_kph
      # The toll held an explicit coast-restore synchronization request.
      # Complete it now instead of silently consuming it as a road baseline.

    if self.camera_pending or CS.gasPressed or target_raw_limit <= 0:
      return min(v_cruise_kph, self.auto_sync_speed) if self.auto_sync_speed is not None else v_cruise_kph

    if target_raw_limit != self.auto_prev_limit or self.auto_mode_needs_sync:
      # A coast restore synchronizes from the pre-restore speed rather than
      # treating the backup target as fresh evidence for a larger offset.
      reference = self.auto_sync_speed if self.auto_sync_speed is not None else v_cruise_kph
      raw_offset = (reference - target_raw_limit) / 2.0 if target_raw_limit < reference else self.auto_speed_offset
      rounded_offset = math.floor(raw_offset / 5.0 + 0.5) * 5.0
      target = min(110.0, target_raw_limit + max(self.auto_speed_offset, rounded_offset))
      if target_raw_limit < reference:
        target = min(reference, target)
      self.auto_prev_limit = target_raw_limit
      self.auto_mode_needs_sync = False
      self.auto_sync_speed = None
      if target != v_cruise_kph:
        self._add_log(f"Auto speed: {target_raw_limit:g} -> {target:g} km/h")
        self._auto_mode_prompt(f"오토 속도 변경: {int(target_raw_limit)}km/h 🔔")
      return target
    return v_cruise_kph

  ## desiredSpeed :
  #   leadCar_distance, leadCar_speed, leadCar_accel,
  #   v_ego, tbt_distance, tbt_speed,
  #   nRoadLimitSpeed, vTurnSpeed
  #   gasPressed, brakePressed, standstill
  def _v_cruise_desired(self, CS, v_cruise_kph):
    if self._cruise_button_mode == 3:
      for speed in self._cruise_speed_table:
        if v_cruise_kph < speed:
          v_cruise_kph = speed
          break
      else:
        v_cruise_kph = ((v_cruise_kph // self._cruise_speed_unit) + 1) * self._cruise_speed_unit

    elif v_cruise_kph < 30: #self.nRoadLimitSpeed:
      v_cruise_kph = 30 #self.nRoadLimitSpeed
    else:
      for speed in range (40, 160, self._cruise_speed_unit):
        if v_cruise_kph < speed:
          v_cruise_kph = speed
          break
    return v_cruise_kph

  def _auto_speed_up(self, v_cruise_kph):
    if self.current_driving_mode == 5:
      # Mode 5 owns its target. Lead/model/road auto-speed paths must not
      # bypass its camera hold, toll priority or automatic speed ceiling.
      self.road_limit_kph = self.nRoadLimitSpeed * self.autoSpeedUptoRoadSpeedLimit
      self.nRoadLimitSpeed_last = self.nRoadLimitSpeed
      return v_cruise_kph
    #if self._pause_auto_speed_up:
    #  return v_cruise_kph
    if not self._pause_auto_speed_up and self.applyModelSpeed != 0.0:
      model_kph = self.model_v_kph * abs(self.applyModelSpeed)
      apply_speed = min(model_kph, self.nRoadLimitSpeed * 1.1)

      if self.applyModelSpeed < 0.0:
        v_cruise_kph = min(model_kph, apply_speed)
      elif v_cruise_kph < apply_speed:
        v_cruise_kph = apply_speed

    road_limit_kph = self.nRoadLimitSpeed * self.autoSpeedUptoRoadSpeedLimit
    if road_limit_kph < 1.0:
      return v_cruise_kph

    #if self.autoRoadSpeedLimitOffset > 0:
      #self._v_cruise_kph_at_brake = self.nRoadLimitSpeed + self.autoRoadSpeedLimitOffset

    if not self._pause_auto_speed_up and self.v_lead_kph + 5 > v_cruise_kph and v_cruise_kph < road_limit_kph and self.d_rel < 60:
      v_cruise_kph = min(v_cruise_kph + 5, road_limit_kph)
    elif self.autoRoadSpeedAdjust < 0 and self.nRoadLimitSpeed != self.nRoadLimitSpeed_last:  # 도로제한속도가 바뀌면, 바뀐속도로 속도를 바꿈.
      # ▼▼▼ [버그 픽스] 오토모드(5번)이거나 카메라 안내 중이면 여기서 멋대로 속도를 내리지 않음! ▼▼▼
      if getattr(self, 'current_driving_mode', 3) == 5 or getattr(self, 'is_cam', False):
        pass
      elif self.autoRoadSpeedLimitOffset < 0:
        v_cruise_kph = self.nRoadLimitSpeed * self.autoNaviSpeedSafetyFactor
      else:
        v_cruise_kph = self.nRoadLimitSpeed + self.autoRoadSpeedLimitOffset
    elif self.nRoadLimitSpeed < self.nRoadLimitSpeed_last and self.autoRoadSpeedAdjust > 0:
      # ▼▼▼ [버그 픽스] 오토모드(5번)이거나 카메라 안내 중이면 여기서 멋대로 속도를 내리지 않음! ▼▼▼
      if getattr(self, 'current_driving_mode', 3) == 5 or getattr(self, 'is_cam', False):
        pass
      else:
        new_road_limit_kph = self.nRoadLimitSpeed * self.autoRoadSpeedAdjust + v_cruise_kph * (1 - self.autoRoadSpeedAdjust)
        self._add_log(f"AutoSpeed change {v_cruise_kph} -> {new_road_limit_kph:.1f}")
        v_cruise_kph = min(v_cruise_kph, new_road_limit_kph)
        
    self.road_limit_kph = road_limit_kph
    self.nRoadLimitSpeed_last = self.nRoadLimitSpeed
    return v_cruise_kph

  def _cruise_control(self, enable, cancel_timer, reason):
    if self._cruise_cancel_state: # and self._soft_hold_active != 2:
      self._add_log(reason + " > Cancel state")
    elif enable > 0 and self._cancel_timer > 0 and cancel_timer >= 0:
      enable = 0
      self._add_log(reason + " > Canceled")
    else:
      if self.autoCruiseControl == 0 and enable != 0:
        enable = 0
        self._soft_hold_active = 0
        return
      if self.autoCruiseControl_cancel_timer > 0 and enable != 0:
        self._add_log(reason + " > timer Canceled")
        enable = 0
        self._soft_hold_active = 0
        return
      self._activate_cruise = enable
      self._cancel_timer = int(cancel_timer / 0.01)   # DT_CTRL: 0.01
      self._add_log(reason)

  def _check_safe_stop(self, CS, safe_distance = 3):
    v_ego = CS.vEgo
    decel_rate = 1.5
    d_stop_ego = (v_ego ** 2) / (2 * decel_rate)
    d_stop_rel = (self.v_rel ** 2) / (2 * decel_rate)

    d_final = self.d_rel - d_stop_ego - d_stop_rel

    if d_final >= safe_distance:
      return True, d_final
    else:
      return False, d_final

  def _update_cruise_state(self, CS, CC, v_cruise_kph):
    if not CC.enabled:
      #self._pause_auto_speed_up = False
      if self._brake_pressed_count == -1 and self._soft_hold_active > 0:
        self._soft_hold_active = 2
        #self.autoCruiseControl_cancel_timer = 0
        self._cruise_control(1, -1, "Cruise on (soft hold)")
      # GM: autoResume
      elif self.params.get_bool("ActivateCruiseAfterBrake"):
        self.params.put_bool_nonblocking("ActivateCruiseAfterBrake", False)
        self._cruise_control(1, -1, "Cruise on (brake)")
      elif self.v_cruise_kph < self.v_ego_kph_set:
        self.v_cruise_kph = self.v_ego_kph_set

    if self._soft_hold_active > 0:
      #self.events.append(EventName.softHold)
      #self._cruise_cancel_state = False
      pass

    if self._gas_tok and self.v_ego_kph_set >= self.autoGasTokSpeed:
      if not CC.enabled:
        #self._cruise_cancel_state = False
        self._cruise_control(1, -1, "Cruise on (gas tok)")
        if self.v_ego_kph_set > v_cruise_kph:
          v_cruise_kph = self.v_ego_kph_set
      else:
        v_cruise_kph = self._v_cruise_desired(CS, v_cruise_kph)
    elif self._gas_pressed_count == -1:
      if 0 < self.d_rel < CS.vEgo * 0.8:
        if CS.vEgo < 1.0:
          self._cruise_control(1, -1 if self.aTarget > 0.0 else 0, "Cruise on (safe speed)")
        else:
          self._cruise_control(-1, 0, "Cruise off (lead car too close)")
      elif self.v_ego_kph_set < 30:
        self._cruise_control(-1, 0, "Cruise off (gas speed)")
      elif self.xState == 3:
        v_cruise_kph = min(self.v_ego_kph_set, v_cruise_kph)
        self._cruise_control(-1, 3, "Cruise off (traffic sign)")
      elif self.v_ego_kph_set >= self.autoGasTokSpeed and not CC.enabled:
        v_cruise_kph = min(self.v_ego_kph_set, v_cruise_kph)
        self._cruise_control(1, -1 if self.aTarget > 0.0 else 0, "Cruise on (gas pressed)")
    elif self._brake_pressed_count == -1 and self._soft_hold_active == 0:
      if self.v_ego_kph_set > self.autoGasTokSpeed:
        v_cruise_kph = self.v_ego_kph_set
        self._cruise_control(1, -1 if self.aTarget > 0.0 else 0, "Cruise on (speed)")
      elif abs(CS.steeringAngleDeg) < 20:
        if self.xState in [3, 5]:
          if self.xState == 3:  # 감속중
            v_cruise_kph = self.v_ego_kph_set
          self._cruise_control(1, 0, "Cruise on (traffic sign)")
        elif 0 < self.d_rel < 20: 
          # v_cruise_kph = self.v_ego_kph_set # 전방에 차가 가까이 있을때, 기존속도 유지
          self._cruise_control(1, -1 if self.v_ego_kph_set < 1 else 0, "Cruise on (lead car)")

    elif self._brake_pressed_count < 0 and self._gas_pressed_count < 0:
      if not CC.enabled:
        if self.d_rel > 0 and CS.vEgo > 0.02:
          safe_state, safe_dist = self._check_safe_stop(CS, 4)
          if abs(CS.steeringAngleDeg) > 70:
            pass
          elif not safe_state:
            self._cruise_control(1, -1, "Cruise on (fcw)")
          elif self.d_rel < self.cruiseOnDist:
            self._cruise_control(1, 0, "Cruise on (fcw dist)")
          else:
            self._add_log(f"leadCar d={self.d_rel:.1f},v={self.v_rel:.1f},{CS.vEgo:.1f}, {safe_dist:.1f}")
            #self.events.append(EventName.stopStop)
        if self.desiredSpeed < self.v_ego_kph_set:
          self._cruise_control(1, -1, "Cruise on (desired speed)")
        if self._cruise_ready:
          if self.xState in [3]:
            self._cruise_control(1, 0, "Cruise on (traffic sign)")
          elif self.d_rel > 0:
            self._cruise_control(1, 0, "Cruise on (lead car)")
      elif self._paddle_decel_active:
        if self.xState in [3]:
          self._paddle_decel_active = False
          v_cruise_kph = self.v_ego_kph_set          

    if self._gas_pressed_count > self._gas_tok_timer:
      if CS.aEgo < -0.5:
        self._cruise_control(-1, 5.0, "Cruise off (gas pressed while braking)")
      if self.v_ego_kph_set > v_cruise_kph and self.autoGasSyncSpeed:
        v_cruise_kph = self.v_ego_kph_set

    if self._gas_pressed_count == 1 or CS.vEgo < 0.1:
      self._pause_auto_speed_up = False
      if self._gas_pressed_count == 1 and CS.vEgo < 0.1:
        self._cruise_control(-1, -1, "Cruise off (gasPressed)")
    elif self._brake_pressed_count > 0:
      self._pause_auto_speed_up = True

    # ▼▼▼ [수정 2] 당근크루즈 중 타력주행 유지 및 확실한 감속 보장 ▼▼▼
    if getattr(self, 'carrot_cruise_active', False):
      # 1. 엑셀을 밟지 않을 때 강제 가속 방지 (타력 주행)
      # 💡 목표 속도(v_cruise_kph)가 현재 속도보다 높으면 안 됨 (가속 차단)
      # 2. 확실한 제동 보장
      # 💡 앞차를 보고 모델이 요구하는 감속 속도(v_cruise_kph)가 현재 속도보다 낮으면 그 값을 그대로 허용!
      v_cruise_kph = min(v_cruise_kph, self.v_ego_kph_set) 
      self._pause_auto_speed_up = True                     
    # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

    return self._auto_speed_up(v_cruise_kph)

  def _prepare_brake_gas(self, CS, CC):
    if CS.gasPressed:
      self._paddle_decel_active = False
      self._gas_pressed_count = max(1, self._gas_pressed_count + 1)
      self._gas_pressed_count_last = self._gas_pressed_count
      self._gas_pressed_value = max(CS.gas, self._gas_pressed_value) if self._gas_pressed_count > 1 else CS.gas
      self._gas_tok = False
      #if self._cruise_cancel_state and self._soft_hold_active == 2:
      #  self._cruise_control(-1, -1, "Cruise off,softhold mode (gasPressed)")
      self._soft_hold_active = 0
    else:
      self._gas_tok = True if 0 < self._gas_pressed_count < self._gas_tok_timer else False
      self._gas_pressed_count = min(-1, self._gas_pressed_count - 1)
      if self._gas_pressed_count < -1:
        self._gas_pressed_count_last = 0
        self._gas_tok = False

    if CS.brakePressed:
      self._cruise_ready = False
      self._paddle_decel_active = False
      if self.carrot_cruise_active and self._carrot_cruise_saved_op_speed is not None:
        # The regular brake-resume backup must capture the prior OpenPilot
        # setpoint, not a zero speed produced by the coasting clamp.
        self._v_cruise_kph_at_brake = self._carrot_cruise_saved_op_speed
      self.carrot_cruise_active = False
      self._brake_pressed_count = max(1, self._brake_pressed_count + 1)
      if self._brake_pressed_count == 1 and self.enabled_last:
        # ▼▼▼ [수정] 금고가 비어있을 때만 백업! (당근크루즈가 넣어둔 80 보존) ▼▼▼
        if getattr(self, '_v_cruise_kph_at_brake', 0) == 0:
          self._v_cruise_kph_at_brake = self.v_cruise_kph
        self._add_log(f"{self._v_cruise_kph_at_brake} Cruise speed at brake")
        # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲
      self._soft_hold_count = self._soft_hold_count + 1 if CS.vEgo < 0.1 and CS.gearShifter == GearShifter.drive else 0
      if self.autoCruiseControl == 0 or self.CP.pcmCruise:
        self._soft_hold_active = 0
      else:
        self._soft_hold_active = 1 if self._soft_hold_count > 60 else 0
    else:
      self._soft_hold_count = 0
      self._brake_pressed_count = min(-1, self._brake_pressed_count - 1)
