from collections import deque
import copy
import math
import numpy as np
import ast

from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, create_button_events, structs, DT_CTRL
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, CAR, DBC, Buttons, CarControllerParams, CAMERA_SCC_CAR, HyundaiExtFlags
from opendbc.car.interfaces import CarStateBase

from openpilot.common.params import Params

from datetime import datetime
from zoneinfo import ZoneInfo


ButtonType = structs.CarState.ButtonEvent.Type

PREV_BUTTON_SAMPLES = 8
CLUSTER_SAMPLE_RATE = 20  # frames
STANDSTILL_THRESHOLD = 12 * 0.03125 * CV.KPH_TO_MS

# ▼▼▼ [추가] 방지턱 전용 상수 ▼▼▼
VEHICLE_NAVI_MAX_EVENT_DISTANCE = 2500.0
VEHICLE_NAVI_PASSED_EVENT_DISTANCE = 30.0
VEHICLE_NAVI_MAX_EVENTS = 32
# ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

BUTTONS_DICT = {Buttons.RES_ACCEL: ButtonType.accelCruise, Buttons.SET_DECEL: ButtonType.decelCruise,
                Buttons.GAP_DIST: ButtonType.gapAdjustCruise, Buttons.CANCEL: ButtonType.cancel, Buttons.LFA_BUTTON: ButtonType.lfaButton}

GearShifter = structs.CarState.GearShifter
READY_COUNT_OK = 200


NUMERIC_TO_TZ = {
    840: "America/New_York",   # 미국 (US) → 동부 시간대
    124: "America/Toronto",    # 캐나다 (CA) → 동부 시간대
    250: "Europe/Paris",       # 프랑스 (FR)
    276: "Europe/Berlin",      # 독일 (DE)
    826: "Europe/London",      # 영국 (GB)
    392: "Asia/Tokyo",         # 일본 (JP)
    156: "Asia/Shanghai",      # 중국 (CN)
    410: "Asia/Seoul",         # 한국 (KR)
     36: "Australia/Sydney",   # 호주 (AU)
    356: "Asia/Kolkata",       # 인도 (IN)
}

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])

    self.cruise_buttons: deque = deque([Buttons.NONE] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES)
    self.main_buttons: deque = deque([Buttons.NONE] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES)

    self.gear_msg_canfd = "GEAR" if CP.extFlags & HyundaiExtFlags.CANFD_GEARS_69 else \
                          "ACCELERATOR" if CP.flags & HyundaiFlags.EV else \
                          "GEAR_ALT" if CP.flags & HyundaiFlags.CANFD_ALT_GEARS else \
                          "GEAR_ALT_2" if CP.flags & HyundaiFlags.CANFD_ALT_GEARS_2 else \
                          "GEAR_SHIFTER"

    self.use_accelerator = self.gear_msg_canfd == "ACCELERATOR"
    if CP.flags & HyundaiFlags.CANFD:
      self.shifter_values = can_define.dv[self.gear_msg_canfd]["GEAR"]
    elif CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
      self.shifter_values = can_define.dv["ELECT_GEAR"]["Elect_Gear_Shifter"]
    elif self.CP.flags & HyundaiFlags.CLUSTER_GEARS:
      self.shifter_values = can_define.dv["CLU15"]["CF_Clu_Gear"]
    elif self.CP.flags & HyundaiFlags.TCU_GEARS:
      self.shifter_values = can_define.dv["TCU12"]["CUR_GR"]
    elif CP.flags & HyundaiFlags.FCEV:
      self.shifter_values = can_define.dv["EMS20"]["HYDROGEN_GEAR_SHIFTER"]
    else:
      self.shifter_values = can_define.dv["LVR12"]["CF_Lvr_Gear"]

    self.accelerator_msg_canfd = "ACCELERATOR" if CP.flags & HyundaiFlags.EV else \
                                 "ACCELERATOR_ALT" if CP.flags & HyundaiFlags.HYBRID else \
                                 "ACCELERATOR_BRAKE_ALT"
    self.cruise_btns_msg_canfd = "CRUISE_BUTTONS_ALT" if CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS else \
                                 "CRUISE_BUTTONS"
    self.is_metric = False
    self.buttons_counter = 0

    # for generic CAN parsing
    self.fca11 = None
    self.scc11 = None
    self.scc12 = None
    self.scc13 = None
    self.scc14 = None
    self.lkas11 = None
    self.clu11 = None
    
    # for CANFD parsing
    self.scc_control = None    
    self.lfa = None    
    self.lfa_alt = None    
    self.lfahda_cluster = None    
    self.adrv_0x161 = None
    self.adrv_0x200 = None
    self.adrv_0x1ea = None
    self.adrv_0x160 = None
    self.ccnc_0x162 = None    
    self.hda_info_4a3 = None    
    
    # ▼▼▼ [추가] 순정 내비 방지턱 트래킹 초기화 ▼▼▼
    self.navi_segment_4b9 = None
    self.navi_profile_4be = None
    self.vehicleNaviEvents = []
    self.vehicleNaviSegmentTimestamp = 0
    self.vehicleNaviProfileTimestamp = 0
    self.vehicleNaviRouteResetTimestamp = 0
    self.frame_for_params = 0
    self.vehicleNaviCanControl = False
    # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

    self.tcs = None    
    self.mdps = None
    self.steer_touch_2af = None
    self.cruise_buttons_msg = None
    self.cam_0x362 = None
    self.cam_0x2a4 = None
    self.manual_speed_limit_assist = None
    self.accelerator = None
    self.blinkers = None
    self.blinkers_alt = None  # 추가
    self.blinker_stalks = None  # 추가
    self.left_stalk_prev = self.right_stalk_prev = False  # 추가
    self.left_blinker_stalk_count = self.right_blinker_stalk_count = 0  # 추가
    self.doors_seatbelts = None
    self.cruise_buttons_alt2 = None

    # On some cars, CLU15->CF_Clu_VehicleSpeed can oscillate faster than the dash updates. Sample at 5 Hz
    self.cluster_speed = 0
    self.cluster_speed_counter = CLUSTER_SAMPLE_RATE

    self.params = CarControllerParams(CP)

    self.main_enabled = True if Params().get_int("AutoEngage") == 2 else False
    self.gear_shifter = GearShifter.drive # Gear_init for Nexo ?? unknown 21.02.23.LSW

    self.totalDistance = 0.0
    self.speedLimitDistance = 0
    self.pcmCruiseGap = 0

    self.cruise_buttons_alt =  True if self.CP.carFingerprint in (CAR.HYUNDAI_CASPER, CAR.HYUNDAI_CASPER_EV) else False
    self.MainMode_ACC = False
    self.ACCMode = 0
    self.LFA_ICON = 0
    self.paddle_button_prev = 0

    self.lf_distance = 0
    self.rf_distance = 0
    self.lr_distance = 0
    self.rr_distance = 0
    #self.lf_lateral = 0
    #self.rf_lateral = 0

    fingerprints_str = Params().get("FingerPrints")
    fingerprints = ast.literal_eval(fingerprints_str)
    #print("fingerprints =", fingerprints)
    ecu_disabled = False
    if self.CP.openpilotLongitudinalControl and not (self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC):
      ecu_disabled = True

    
    self.HAS_LFA_BUTTON = True if 913 in fingerprints[0] else False
    self.CRUISE_BUTTON_ALT = True if 1007 in fingerprints[0] else False

    cam_bus = CanBus(CP).CAM
    pt_bus = CanBus(CP).ECAN
    alt_bus = CanBus(CP).ACAN
    self.GEAR = True if 69 in fingerprints[pt_bus] else False
    self.GEAR_ALT = True if 64 in fingerprints[pt_bus] else False
    self.TPMS = True if 0x3a0 in fingerprints[pt_bus] else False
    self.LOCAL_TIME = True if 1264 in fingerprints[pt_bus] else False

    self.cp_bsm = None
    self.time_zone = "UTC"
    
    self.cp = None
    self.cp_cam = None
    self.cp_alt = None
    self.controls_ready_count = 0

  def monitor_fingerprint(self, can_parsers, canfd):
    if self.controls_ready_count <= READY_COUNT_OK:
      if Params().get_bool("ControlsReady"):
        self.controls_ready_count += 1
      self.cp = can_parsers[Bus.pt]
      self.cp_cam = can_parsers[Bus.cam]
      self.cp_alt = can_parsers[Bus.alt] if Bus.alt in can_parsers else None

      def add_if_seen(parser, name, ignore_counter = False):
        msg = parser.dbc.name_to_msg.get(name)
        if not msg:
          print(f"{name} not in DBC")
          return
        if msg.address not in parser.seen_addresses:
          return
        if msg.address in parser.addresses:
          return
        parser._add_message(name, ignore_counter = ignore_counter)   # ← 이름으로 등록

      def add_and_cache(parser, name: str, attr: str, ignore_counter: bool = False):
        add_if_seen(parser, name, ignore_counter)
        if name in parser.vl:   # 등록 성공했을 때만
          setattr(self, attr, parser.vl[name])
          return True
        return False
      
      if self.controls_ready_count == 50:
        self.cp.controls_ready = self.cp_cam.controls_ready = True
        if self.cp_alt is not None:
          self.cp_alt.controls_ready = True
      elif self.controls_ready_count == 100:
        self.cp.enable_capture = self.cp_cam.enable_capture = False
        if self.cp_alt is not None:
          self.cp_alt.enable_capture = False
      elif self.controls_ready_count == 101:
        print("cp_cam.seen_addresses =", self.cp_cam.seen_addresses)
      elif self.controls_ready_count == 102:
        print("cp.seen_addresses =", self.cp.seen_addresses)
      elif self.controls_ready_count == 103:
        if self.cp_alt is not None:
          print("cp_alt.seen_addresses =", self.cp_alt.seen_addresses)
        else:
          print("cp_alt.seen_addresses = None")
      if not canfd:
        if self.controls_ready_count == 104:
          if not add_and_cache(self.cp_cam, "FCA11", "fca11"):
            add_and_cache(self.cp, "FCA11", "fca11")
          add_and_cache(self.cp_cam, "LKAS11", "lkas11")
          add_and_cache(self.cp, "CLU11", "clu11")       
        elif self.controls_ready_count == 105:
          cp_cruise = self.cp_cam if self.CP.flags & HyundaiFlags.CAMERA_SCC else self.cp
          add_and_cache(cp_cruise, "SCC11", "scc11")
          add_and_cache(cp_cruise, "SCC12", "scc12")
          add_and_cache(cp_cruise, "SCC13", "scc13")
          add_and_cache(cp_cruise, "SCC14", "scc14")
      else: # canfd
        if self.controls_ready_count == 120:
          cp_cruise = self.cp_cam if self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC else self.cp
          add_and_cache(cp_cruise, "SCC_CONTROL", "scc_control")          
        elif self.controls_ready_count == 121:
          add_and_cache(self.cp, "TCS", "tcs")
          add_and_cache(self.cp, "MDPS", "mdps")
          add_and_cache(self.cp_cam, "LFA", "lfa")
          add_and_cache(self.cp_cam, "LFA_ALT", "lfa_alt")          
          add_and_cache(self.cp_cam, "LFAHDA_CLUSTER", "lfahda_cluster")
        elif self.controls_ready_count == 122:
          add_and_cache(self.cp_cam, "ADRV_0x161", "adrv_0x161")  
          add_and_cache(self.cp_cam, "ADRV_0x200", "adrv_0x200")
          add_and_cache(self.cp_cam, "ADRV_0x1ea", "adrv_0x1ea")
          add_and_cache(self.cp_cam, "ADRV_0x160", "adrv_0x160")
          add_and_cache(self.cp_cam, "CCNC_0x162", "ccnc_0x162")
        elif self.controls_ready_count == 123:        
          add_and_cache(self.cp, "HDA_INFO_4A3", "hda_info_4a3")
          
          # ▼▼▼ [추가] 4B9, 4BE 방지턱 메시지 허용 ▼▼▼
          add_and_cache(self.cp, "NEW_MSG_4B9", "navi_segment_4b9")
          add_and_cache(self.cp, "NEW_MSG_4BE", "navi_profile_4be")
          # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲
          
          add_and_cache(self.cp, "STEER_TOUCH_2AF", "steer_touch_2af")
        elif self.controls_ready_count == 124:
          add_and_cache(self.cp, self.cruise_btns_msg_canfd, "cruise_buttons_msg")
          if not add_and_cache(self.cp_cam, "CAM_0x362", "cam_0x362") and self.cp_alt is not None:
            add_and_cache(self.cp_alt, "CAM_0x362", "cam_0x362")
          if not add_and_cache(self.cp_alt, "CAM_0x2a4", "cam_0x2a4") and self.cp_cam is not None:
            add_and_cache(self.cp_cam, "CAM_0x2a4", "cam_0x2a4")
        elif self.controls_ready_count == 125:
          add_and_cache(self.cp, "MANUAL_SPEED_LIMIT_ASSIST", "manual_speed_limit_assist", ignore_counter = True)
          if self.gear_msg_canfd == "ACCELERATOR":
            add_and_cache(self.cp, "ACCELERATOR", "accelerator", ignore_counter = True)
          add_and_cache(self.cp, "BLINKERS", "blinkers")
          add_and_cache(self.cp, "BLINKERS_ALT", "blinkers_alt")  # 추가
          add_and_cache(self.cp, "BLINKER_STALKS", "blinker_stalks", ignore_counter=True)  # 추가
          add_and_cache(self.cp, "DOORS_SEATBELTS", "doors_seatbelts")
        elif self.controls_ready_count == 126:
          add_and_cache(self.cp, "CRUISE_BUTTONS_ALT2", "cruise_buttons_alt2", ignore_counter = True)
               
    
  def update(self, can_parsers) -> structs.CarState:
    self.monitor_fingerprint(can_parsers, self.CP.flags & HyundaiFlags.CANFD)
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    cp_alt = can_parsers[Bus.alt] if Bus.alt in can_parsers else None

    if self.CP.flags & HyundaiFlags.CANFD:
      return self.update_canfd(can_parsers)

    ret = structs.CarState()
    cp_cruise = cp_cam if self.CP.flags & HyundaiFlags.CAMERA_SCC else cp
    self.is_metric = cp.vl["CLU11"]["CF_Clu_SPEED_UNIT"] == 0
    speed_conv = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    ret.doorOpen = any([cp.vl["CGW1"]["CF_Gway_DrvDrSw"], cp.vl["CGW1"]["CF_Gway_AstDrSw"],
                        cp.vl["CGW2"]["CF_Gway_RLDrSw"], cp.vl["CGW2"]["CF_Gway_RRDrSw"]])

    ret.seatbeltUnlatched = cp.vl["CGW1"]["CF_Gway_DrvSeatBeltSw"] == 0

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHL_SPD11"]["WHL_SPD_FL"],
      cp.vl["WHL_SPD11"]["WHL_SPD_FR"],
      cp.vl["WHL_SPD11"]["WHL_SPD_RL"],
      cp.vl["WHL_SPD11"]["WHL_SPD_RR"],
    )
    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.wheelSpeeds.fl <= STANDSTILL_THRESHOLD and ret.wheelSpeeds.rr <= STANDSTILL_THRESHOLD

    self.cluster_speed_counter += 1
    if self.cluster_speed_counter > CLUSTER_SAMPLE_RATE:
      self.cluster_speed = cp.vl["CLU15"]["CF_Clu_VehicleSpeed"]
      self.cluster_speed_counter = 0

      # Mimic how dash converts to imperial.
      # Sorento is the only platform where CF_Clu_VehicleSpeed is already imperial when not is_metric
      # TODO: CGW_USM1->CF_Gway_DrLockSoundRValue may describe this
      if not self.is_metric and self.CP.carFingerprint not in (CAR.KIA_SORENTO,):
        self.cluster_speed = math.floor(self.cluster_speed * CV.KPH_TO_MPH + CV.KPH_TO_MPH)

    #ret.vEgoCluster = self.cluster_speed * speed_conv

    ret.steeringAngleDeg = cp.vl["SAS11"]["SAS_Angle"]
    ret.steeringRateDeg = cp.vl["SAS11"]["SAS_Speed"]
    ret.yawRate = cp.vl["ESP12"]["YAW_RATE"]
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(
      50, cp.vl["CGW1"]["CF_Gway_TurnSigLh"], cp.vl["CGW1"]["CF_Gway_TurnSigRh"])
    ret.steeringTorque = cp.vl["MDPS12"]["CR_Mdps_StrColTq"]
    ret.steeringTorqueEps = cp.vl["MDPS12"]["CR_Mdps_OutTq"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > self.params.STEER_THRESHOLD, 5)
    ret.steerFaultTemporary = cp.vl["MDPS12"]["CF_Mdps_ToiUnavail"] != 0 or cp.vl["MDPS12"]["CF_Mdps_ToiFlt"] != 0

    # cruise state
    if self.CP.openpilotLongitudinalControl:
      # These are not used for engage/disengage since openpilot keeps track of state using the buttons
      ret.cruiseState.available = self.main_enabled and self.controls_ready_count >= READY_COUNT_OK #cp.vl["TCS13"]["ACCEnable"] == 0
      ret.cruiseState.enabled = cp.vl["TCS13"]["ACC_REQ"] == 1
      ret.cruiseState.standstill = False
      ret.cruiseState.nonAdaptive = False
    elif not self.CP.flags & HyundaiFlags.CC_ONLY_CAR:
      self.main_enabled = ret.cruiseState.available = cp_cruise.vl["SCC11"]["MainMode_ACC"] == 1
      ret.cruiseState.enabled = cp_cruise.vl["SCC12"]["ACCMode"] != 0
      ret.cruiseState.standstill = cp_cruise.vl["SCC11"]["SCCInfoDisplay"] == 4.
      ret.cruiseState.nonAdaptive = cp_cruise.vl["SCC11"]["SCCInfoDisplay"] == 2.  # Shows 'Cruise Control' on dash
      ret.cruiseState.speed = cp_cruise.vl["SCC11"]["VSetDis"] * speed_conv

      ret.pcmCruiseGap = cp_cruise.vl["SCC11"]["TauGapSet"]

    # TODO: Find brake pressure
    ret.brake = 0
    if not self.CP.flags & HyundaiFlags.CC_ONLY_CAR:
      ret.brakePressed = cp.vl["TCS13"]["DriverOverride"] == 2  # 2 includes regen braking by user on HEV/EV
      ret.brakeHoldActive = cp.vl["TCS15"]["AVH_LAMP"] == 2  # 0 OFF, 1 ERROR, 2 ACTIVE, 3 READY
      ret.parkingBrake = cp.vl["TCS13"]["PBRAKE_ACT"] == 1
      ret.espDisabled = cp.vl["TCS11"]["TCS_PAS"] == 1
      ret.espActive = cp.vl["TCS11"]["ABS_ACT"] == 1
      ret.accFaulted = cp.vl["TCS13"]["ACCEnable"] != 0  # 0 ACC CONTROL ENABLED, 1-3 ACC CONTROL DISABLED
      ret.brakeLights = bool(cp.vl["TCS13"]["BrakeLight"] or ret.brakePressed)

    if self.CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV | HyundaiFlags.FCEV):
      if self.CP.flags & HyundaiFlags.FCEV:
        ret.gas = cp.vl["FCEV_ACCELERATOR"]["ACCELERATOR_PEDAL"] / 254.
      elif self.CP.flags & HyundaiFlags.HYBRID:
        ret.gas = cp.vl["E_EMS11"]["CR_Vcu_AccPedDep_Pos"] / 254.
      else:
        ret.gas = cp.vl["E_EMS11"]["Accel_Pedal_Pos"] / 254.
      ret.gasPressed = ret.gas > 0
    else:
      ret.gas = cp.vl["EMS12"]["PV_AV_CAN"] / 100.
      ret.gasPressed = bool(cp.vl["EMS16"]["CF_Ems_AclAct"])

    # Gear Selection via Cluster - For those Kia/Hyundai which are not fully discovered, we can use the Cluster Indicator for Gear Selection,
    # as this seems to be standard over all cars, but is not the preferred method.
    if self.CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
      gear = cp.vl["ELECT_GEAR"]["Elect_Gear_Shifter"]
      ret.gearStep = cp.vl["ELECT_GEAR"]["Elect_Gear_Step"]
    elif self.CP.flags & HyundaiFlags.FCEV:
      gear = cp.vl["EMS20"]["HYDROGEN_GEAR_SHIFTER"]
    elif self.CP.flags & HyundaiFlags.CLUSTER_GEARS:
      gear = cp.vl["CLU15"]["CF_Clu_Gear"]
    elif self.CP.flags & HyundaiFlags.TCU_GEARS:
      gear = cp.vl["TCU12"]["CUR_GR"]
    else:
      gear = cp.vl["LVR12"]["CF_Lvr_Gear"]
      ret.gearStep = cp.vl["LVR11"]["CF_Lvr_GearInf"]

    if not self.CP.carFingerprint in (CAR.HYUNDAI_NEXO):
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))
    else:
      gear = cp.vl["ELECT_GEAR"]["Elect_Gear_Shifter"]
      gear_disp = cp.vl["ELECT_GEAR"]

      gear_shifter = GearShifter.unknown

      if gear == 1546:  # Thank you for Neokii  # fix PolorBear 22.06.05
        gear_shifter = GearShifter.drive
      elif gear == 2314:
        gear_shifter = GearShifter.neutral
      elif gear == 2569:
        gear_shifter = GearShifter.park
      elif gear == 2566:
        gear_shifter = GearShifter.reverse

      if gear_shifter != GearShifter.unknown and self.gear_shifter != gear_shifter:
        self.gear_shifter = gear_shifter

      ret.gearShifter = self.gear_shifter

    if not self.CP.flags & HyundaiFlags.CC_ONLY_CAR and (not self.CP.openpilotLongitudinalControl or self.CP.flags & HyundaiFlags.CAMERA_SCC):
      aeb_src = "FCA11" if self.CP.flags & HyundaiFlags.USE_FCA.value else "SCC12"
      aeb_sig = "FCA_CmdAct" if self.CP.flags & HyundaiFlags.USE_FCA.value else "AEB_CmdAct"
      aeb_warning = cp_cruise.vl[aeb_src]["CF_VSM_Warn"] != 0
      scc_warning = cp_cruise.vl["SCC12"]["TakeOverReq"] == 1  # sometimes only SCC system shows an FCW
      aeb_braking = cp_cruise.vl[aeb_src]["CF_VSM_DecCmdAct"] != 0 or cp_cruise.vl[aeb_src][aeb_sig] != 0
      ret.stockFcw = (aeb_warning or scc_warning) and not aeb_braking
      ret.stockAeb = aeb_warning and aeb_braking

    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["LCA11"]["CF_Lca_IndLeft"] != 0
      ret.rightBlindspot = cp.vl["LCA11"]["CF_Lca_IndRight"] != 0

    self.steer_state = cp.vl["MDPS12"]["CF_Mdps_ToiActive"]  # 0 NOT ACTIVE, 1 ACTIVE
    prev_cruise_buttons = self.cruise_buttons[-1]
    #self.cruise_buttons.extend(cp.vl_all["CLU11"]["CF_Clu_CruiseSwState"])
    #carrot {{
    #if self.CRUISE_BUTTON_ALT and cp.vl["CRUISE_BUTTON_ALT"]["SET_ME_1"] == 1:
    #  self.cruise_buttons_alt = True

    cruise_button = [Buttons.NONE]
    if self.cruise_buttons_alt:
      lfa_button = cp.vl["CRUISE_BUTTON_LFA"]["CruiseSwLfa"]
      cruise_button = [Buttons.LFA_BUTTON] if lfa_button > 0 else [cp.vl["CRUISE_BUTTON_ALT"]["CruiseSwState"]]
    elif self.HAS_LFA_BUTTON and cp.vl["BCM_PO_11"]["LFA_Pressed"] == 1:  # for K5
      cruise_button = [Buttons.LFA_BUTTON]
    else:
      cruise_button = cp.vl_all["CLU11"]["CF_Clu_CruiseSwState"]
    self.cruise_buttons.extend(cruise_button)
    # }} carrot
    prev_main_buttons = self.main_buttons[-1]
    #self.cruise_buttons.extend(cp.vl_all["CLU11"]["CF_Clu_CruiseSwState"])
    if self.cruise_buttons_alt:
      self.main_buttons.extend(cp.vl_all["CRUISE_BUTTON_ALT"]["CruiseSwMain"])
    else:
      self.main_buttons.extend(cp.vl_all["CLU11"]["CF_Clu_CruiseSwMain"])
    self.mdps12 = copy.copy(cp.vl["MDPS12"])

    ret.buttonEvents = [*create_button_events(self.cruise_buttons[-1], prev_cruise_buttons, BUTTONS_DICT),
                        *create_button_events(self.main_buttons[-1], prev_main_buttons, {1: ButtonType.mainCruise})]


    if not self.CP.flags & HyundaiFlags.CC_ONLY_CAR:
      tpms_unit = cp.vl["TPMS11"]["UNIT"] * 0.725 if int(cp.vl["TPMS11"]["UNIT"]) > 0 else 1.
      ret.tpms.fl = tpms_unit * cp.vl["TPMS11"]["PRESSURE_FL"]
      ret.tpms.fr = tpms_unit * cp.vl["TPMS11"]["PRESSURE_FR"]
      ret.tpms.rl = tpms_unit * cp.vl["TPMS11"]["PRESSURE_RL"]
      ret.tpms.rr = tpms_unit * cp.vl["TPMS11"]["PRESSURE_RR"]

    cluSpeed = cp.vl["CLU11"]["CF_Clu_Vanz"]
    decimal = cp.vl["CLU11"]["CF_Clu_VanzDecimal"]
    if 0. < decimal < 0.5:
      cluSpeed += decimal

    ret.vEgoCluster = cluSpeed * speed_conv
    vEgoClu, aEgoClu = self.update_clu_speed_kf(ret.vEgoCluster)
    ret.vCluRatio = (ret.vEgo / vEgoClu) if (vEgoClu > 3. and ret.vEgo > 3.) else 1.0

    if self.CP.extFlags & HyundaiExtFlags.NAVI_CLUSTER.value:
      speedLimit = cp.vl["Navi_HU"]["SpeedLim_Nav_Clu"]
      speedLimitCam = cp.vl["Navi_HU"]["SpeedLim_Nav_Cam"]
      ret.speedLimit = speedLimit if speedLimit < 255 and speedLimitCam == 1 else 0
      speed_limit_cam = speedLimitCam == 1
    else:
      ret.speedLimit = 0
      ret.speedLimitDistance = 0
      speed_limit_cam = False

    self.update_speed_limit(ret, speed_limit_cam)

    if prev_main_buttons == 0 and self.main_buttons[-1] != 0:
      self.main_enabled = not self.main_enabled

    return ret
  
  # ▼▼▼ [추가] 순정 내비 방지턱 데이터 파싱 및 RAM 디스크 저장 (카메라 무시) ▼▼▼
  def _clear_vehicle_navi_events(self):
    self.vehicleNaviEvents = []

  @staticmethod
  def _vehicle_navi_message_timestamp(cp, name):
    return max(cp.ts_nanos.get(name, {}).values(), default=0)

  @staticmethod
  def _decode_vehicle_navi_segment(values):
    raw = sum(int(values.get(f"BYTE_{i + 1}", 0)) << (i * 8) for i in range(8))
    return {"offset": raw & 0x1fff, "calculated_route": (raw >> 22) & 0x3}

  @staticmethod
  def _decode_vehicle_navi_profile(values):
    return {
      "value": int(values.get("PROLONG_VALUE", 0xffffffff)),
      "offset": int(values.get("PROLONG_OFFSET", 8191)),
      "profile_type": int(values.get("PROLONG_PROFILE_TYPE", 31)),
    }

  @staticmethod
  def _classify_vehicle_navi_profile(profile):
    if profile["profile_type"] != 16:
      return None
      
    val = profile["value"]
    offset = profile.get("offset", 0)

    if val == 6 and 0 < offset <= VEHICLE_NAVI_MAX_EVENT_DISTANCE:
      return "bump", 0, 6

    # ▼▼▼ 카메라 및 국도 제한속도 구역 디코딩 복구 ▼▼▼
    kind = val & 0xF
    speed_code = val >> 4
    if speed_code > 0:
      speed_limit = (speed_code - 1) * 5
      if speed_limit > 0:
        if kind in (0, 1, 2) and 0 < offset <= VEHICLE_NAVI_MAX_EVENT_DISTANCE:
          return "camera", speed_limit, kind
        elif kind == 7:
          return "speed_limit_zone", speed_limit, kind
    return None

  def _add_vehicle_navi_event(self, event_type, speed, kind, offset):
    target = self.totalDistance + offset
    for event in self.vehicleNaviEvents:
      if event["type"] == event_type and abs(event["target"] - target) < 20:
        event["target"] = target
        return
    self.vehicleNaviEvents.append({"type": event_type, "speed": speed, "kind": kind, "target": target})
    self.vehicleNaviEvents.sort(key=lambda event: event["target"])
    self.vehicleNaviEvents = self.vehicleNaviEvents[:VEHICLE_NAVI_MAX_EVENTS]

  
  def _update_blinker_stalks(self, ret):
    """Turn-signal lever (BLINKER_STALKS): presses (rising edges) into carState.*BlinkerStalkCount, position into blinkerLever."""
    if self.blinker_stalks is not None:
      left_stalk, right_stalk = bool(self.blinker_stalks["LEFT_BLINKER"]), bool(self.blinker_stalks["RIGHT_BLINKER"])
      if left_stalk and not self.left_stalk_prev:
        self.left_blinker_stalk_count = (self.left_blinker_stalk_count + 1) % 256
      if right_stalk and not self.right_stalk_prev:
        self.right_blinker_stalk_count = (self.right_blinker_stalk_count + 1) % 256
      self.left_stalk_prev, self.right_stalk_prev = left_stalk, right_stalk
      # *_TAP is set only in the one-touch detent; a latch passes through it for ~0.1 s
      tap = self.blinker_stalks["LEFT_BLINKER_TAP"] or self.blinker_stalks["RIGHT_BLINKER_TAP"]
      ret.blinkerLever = 1 if tap else 2 if (left_stalk or right_stalk) else 0
    ret.leftBlinkerStalkCount = self.left_blinker_stalk_count
    ret.rightBlinkerStalkCount = self.right_blinker_stalk_count

  
  def _update_vehicle_navi_events(self, cp):
    if not getattr(self, 'vehicleNaviCanControl', False):
      return 0.0, 0.0

    if self.navi_segment_4b9 is not None:
      timestamp = self._vehicle_navi_message_timestamp(cp, "NEW_MSG_4B9")
      if timestamp > self.vehicleNaviSegmentTimestamp:
        self.vehicleNaviSegmentTimestamp = timestamp
        if self._decode_vehicle_navi_segment(self.navi_segment_4b9)["calculated_route"] == 2:
          self.vehicleNaviRouteResetTimestamp = timestamp
          self._clear_vehicle_navi_events()

    if self.navi_profile_4be is not None:
      timestamp = self._vehicle_navi_message_timestamp(cp, "NEW_MSG_4BE")
      if timestamp > self.vehicleNaviProfileTimestamp:
        self.vehicleNaviProfileTimestamp = timestamp
        profile = self._decode_vehicle_navi_profile(self.navi_profile_4be)
        event = self._classify_vehicle_navi_profile(profile)
        if event is not None and timestamp > self.vehicleNaviRouteResetTimestamp:
          self._add_vehicle_navi_event(*event, profile.get("offset", 0))

    self.vehicleNaviEvents = [e for e in self.vehicleNaviEvents if e["target"] >= self.totalDistance - VEHICLE_NAVI_PASSED_EVENT_DISTANCE]
    bumps = [e for e in self.vehicleNaviEvents if e["type"] == "bump" and e["target"] > self.totalDistance]
    
    # ▼▼▼ 카메라와 일반구역 데이터 추출 복구 ▼▼▼
    cameras = [e for e in self.vehicleNaviEvents if e["type"] == "camera" and e["target"] > self.totalDistance]
    zones = [e for e in self.vehicleNaviEvents if e["type"] == "speed_limit_zone"]
    
    bump_dist = bumps[0]["target"] - self.totalDistance if bumps else 0.0
    cam_dist = 0.0
    cam_limit = 0.0
    
    # ▼▼▼ [추가] 4BE 카메라 속도별 거리 제한 필터링 ▼▼▼
    valid_cameras = []
    for c in cameras:
      c_dist = c["target"] - self.totalDistance
      c_limit = c["speed"]
      
      # 80 미만은 300m 이하일 때, 80 이상은 600m 이하일 때만 유효한 카메라로 인정!
      if c_limit < 80 and c_dist <= 50:
        valid_cameras.append(c)
      elif c_limit >= 80 and c_dist <= 100:
        valid_cameras.append(c)
        
    if valid_cameras:
      cam_dist = valid_cameras[0]["target"] - self.totalDistance
      cam_limit = valid_cameras[0]["speed"]
    elif zones:
      # 유효한 카메라가 없거나 너무 멀리 있으면 일반 제한속도(zone)를 따름
      cam_dist = 0.0
      cam_limit = zones[-1]["speed"]
    # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲
      
    try:
      with open("/dev/shm/speed_bump_dist", "w") as f:
        f.write(str(bump_dist))
    except Exception:
      pass

    return cam_limit, cam_dist
  # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

  def update_speed_limit(self, ret, speed_limit_cam):
    self.totalDistance += ret.vEgo * DT_CTRL
    
    # [수정] 엑셀(gasPressed)을 밟아도 카메라 거리를 0으로 날리지 않도록 조건 제거!
    if ret.speedLimit > 0 and speed_limit_cam:
      if self.speedLimitDistance <= self.totalDistance:
        self.speedLimitDistance = self.totalDistance + ret.speedLimit * 6
      self.speedLimitDistance = max(self.totalDistance + 1, self.speedLimitDistance)
    else:
      self.speedLimitDistance = self.totalDistance
      
    ret.speedLimitDistance = self.speedLimitDistance - self.totalDistance

  def update_canfd(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    cp_alt = can_parsers[Bus.alt] if Bus.alt in can_parsers else None

    ret = structs.CarState()

    self.is_metric = cp.vl["CRUISE_BUTTONS_ALT"]["DISTANCE_UNIT"] != 1
    speed_factor = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    if self.CP.flags & (HyundaiFlags.EV | HyundaiFlags.HYBRID):
      offset = 255. if self.CP.flags & HyundaiFlags.EV else 1023.
      ret.gas = cp.vl[self.accelerator_msg_canfd]["ACCELERATOR_PEDAL"] / offset if not self.use_accelerator else 0 if self.accelerator is None else self.accelerator["ACCELERATOR_PEDAL"] / offset
      ret.gasPressed = ret.gas > 1e-5
    else:
      ret.gasPressed = bool(cp.vl[self.accelerator_msg_canfd]["ACCELERATOR_PEDAL_PRESSED"]) if not self.use_accelerator else False if self.accelerator is None else bool(self.accelerator["ACCELERATOR_PEDAL_PRESSED"])

    ret.brakePressed = cp.vl["TCS"]["DriverBraking"] == 1
    #print(cp.vl["TCS"], cp.vl_all["TCS"]["DriverBraking"][-10:])

    if self.doors_seatbelts is not None:
      ret.doorOpen = self.doors_seatbelts["DRIVER_DOOR"] == 1
      ret.seatbeltUnlatched = self.doors_seatbelts["DRIVER_SEATBELT"] == 0
        
    gear = cp.vl[self.gear_msg_canfd]["GEAR"] if not self.use_accelerator else 0 if self.accelerator is None else self.accelerator["GEAR"]
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    if self.TPMS:
      tpms_unit = cp.vl["TPMS"]["UNIT"] * 0.725 if int(cp.vl["TPMS"]["UNIT"]) > 0 else 1.
      ret.tpms.fl = tpms_unit * cp.vl["TPMS"]["PRESSURE_FL"]
      ret.tpms.fr = tpms_unit * cp.vl["TPMS"]["PRESSURE_FR"]
      ret.tpms.rl = tpms_unit * cp.vl["TPMS"]["PRESSURE_RL"]
      ret.tpms.rr = tpms_unit * cp.vl["TPMS"]["PRESSURE_RR"]

    # TODO: figure out positions
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_1"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_2"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_3"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_4"],
    )
    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.wheelSpeeds.fl <= STANDSTILL_THRESHOLD and ret.wheelSpeeds.rr <= STANDSTILL_THRESHOLD

    ret.brakeLights = ret.brakePressed or cp.vl["TCS"]["BrakeLight"] == 1 or ret.aEgo < -0.5

    ret.steeringRateDeg = cp.vl["STEERING_SENSORS"]["STEERING_RATE"]

    # steering angle deg값이 이상함. mdps값이 더 신뢰가 가는듯.. torque steering 차량도 확인해야함.
    #ret.steeringAngleDeg = cp.vl["STEERING_SENSORS"]["STEERING_ANGLE"] * -1
    #ret.steeringAngleDeg = cp.vl["MDPS"]["STEERING_ANGLE"] * -1
    if self.CP.flags & HyundaiFlags.ANGLE_CONTROL:
      ret.steeringAngleDeg = cp.vl["MDPS"]["STEERING_ANGLE_2"] * -1
    else:
      ret.steeringAngleDeg = cp.vl["STEERING_SENSORS"]["STEERING_ANGLE"] * -1
    
    ret.steeringTorque = cp.vl["MDPS"]["STEERING_COL_TORQUE"]
    ret.steeringTorqueEps = cp.vl["MDPS"]["STEERING_OUT_TORQUE"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > self.params.STEER_THRESHOLD, 5)
    ret.steerFaultTemporary = cp.vl["MDPS"]["LKA_FAULT"] != 0 or cp.vl["MDPS"]["LFA2_FAULT"] != 0
    #ret.steerFaultTemporary = False

    # ▼▼▼ 레버 상태 업데이트 함수 호출 추가 ▼▼▼
    self._update_blinker_stalks(ret)

    # ▼▼▼ 기존 램프 처리 로직을 BLINKERS_ALT도 지원하도록 보강 ▼▼▼
    blinkers_info = self.blinkers if self.blinkers is not None else self.blinkers_alt if self.blinkers_alt is not None else None
    if blinkers_info is not None:
      left_blinker_lamp = blinkers_info["LEFT_LAMP"] or blinkers_info["LEFT_LAMP_ALT"]
      right_blinker_lamp = blinkers_info["RIGHT_LAMP"] or blinkers_info["RIGHT_LAMP_ALT"]
      ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, left_blinker_lamp, right_blinker_lamp)

    # ▼▼▼ [추가] cruise.py와 동기화하기 위한 우측 깜빡이 20초(2000프레임) 유지 타이머 ▼▼▼
    if not hasattr(self, 'right_blinker_timer'):
      self.right_blinker_timer = 0
      
    if ret.rightBlinker and not ret.leftBlinker:
      self.right_blinker_timer = 2000
    else:
      self.right_blinker_timer = max(0, self.right_blinker_timer - 1)
    # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

    if self.CP.enableBsm:
      if self.cp_bsm is None:
        if 442 in cp.seen_addresses:
          self.cp_bsm = cp
          print("######## BSM in ECAN")
        elif 442 in cp_cam.seen_addresses:
          self.cp_bsm = cp_cam
          print("######## BSM in CAM")
      else:
        bsm_info = self.cp_bsm.vl["BLINDSPOTS_REAR_CORNERS"]
        ret.leftBlindspot = (bsm_info["FL_INDICATOR"] + bsm_info["INDICATOR_LEFT_TWO"] + bsm_info["INDICATOR_LEFT_FOUR"]) > 0
        ret.rightBlindspot = (bsm_info["FR_INDICATOR"] + bsm_info["INDICATOR_RIGHT_TWO"] + bsm_info["INDICATOR_RIGHT_FOUR"]) > 0

    # cruise state
    if self.cruise_buttons_alt2 is not None:
      cruise_button = self.cruise_buttons_alt2["CRUISE_BUTTONS"]
    else:
      cruise_button = cp.vl[self.cruise_btns_msg_canfd]["CRUISE_BUTTONS"]
    if cruise_button in [Buttons.RES_ACCEL, Buttons.SET_DECEL] and self.CP.openpilotLongitudinalControl:
      self.main_enabled = True
    # CAN FD cars enable on main button press, set available if no TCS faults preventing engagement
    ret.cruiseState.available = self.main_enabled and self.controls_ready_count >= READY_COUNT_OK #cp.vl["TCS"]["ACCEnable"] == 0
    if self.CP.flags & HyundaiFlags.CAMERA_SCC.value:
      self.MainMode_ACC = cp_cam.vl["SCC_CONTROL"]["MainMode_ACC"] == 1
      self.ACCMode = cp_cam.vl["SCC_CONTROL"]["ACCMode"]
      self.LFA_ICON = cp_cam.vl["LFAHDA_CLUSTER"]["HDA_LFA_SymSta"]
      
    if self.CP.openpilotLongitudinalControl:
      # These are not used for engage/disengage since openpilot keeps track of state using the buttons
      ret.cruiseState.enabled = cp.vl["TCS"]["ACC_REQ"] == 1
      ret.cruiseState.standstill = False
      if self.MainMode_ACC or self.main_enabled:
        self.main_enabled = True
    else:
      cp_cruise_info = cp_cam if self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC else cp
      ret.cruiseState.enabled = cp_cruise_info.vl["SCC_CONTROL"]["ACCMode"] in (1, 2)
      if cp_cruise_info.vl["SCC_CONTROL"]["MainMode_ACC"] == 1: # carrot
        ret.cruiseState.available = self.main_enabled = True
        ret.pcmCruiseGap = int(np.clip(cp_cruise_info.vl["SCC_CONTROL"]["DISTANCE_SETTING"], 1, 4))
      ret.cruiseState.standstill = cp_cruise_info.vl["SCC_CONTROL"]["InfoDisplay"] >= 4
      ret.cruiseState.speed = cp_cruise_info.vl["SCC_CONTROL"]["VSetDis"] * speed_factor
      ret.brakeHoldActive = cp.vl["ESP_STATUS"]["AUTO_HOLD"] == 1 and cp_cruise_info.vl["SCC_CONTROL"]["ACCMode"] not in (1, 2)

    speed_limit_cam = False
    corner = False
    corner_infos = [info for info in (self.adrv_0x1ea, self.ccnc_0x162) if info is not None]
    if corner_infos:
      def corner_max(signal):
        return max(info[signal] for info in corner_infos)

      ret.leftLongDist = self.lf_distance = corner_max("LF_DETECT_DISTANCE")
      ret.rightLongDist = self.rf_distance = corner_max("RF_DETECT_DISTANCE")
      self.lr_distance = corner_max("LR_DETECT_DISTANCE")
      self.rr_distance = corner_max("RR_DETECT_DISTANCE")
      ret.leftLatDist = corner_max("LF_DETECT_LATERAL")
      ret.rightLatDist = corner_max("RF_DETECT_LATERAL")
      ret.leftRearLongDist = self.lr_distance
      ret.rightRearLongDist = self.rr_distance
      ret.leftRearLatDist = corner_max("LR_DETECT_LATERAL")
      ret.rightRearLatDist = corner_max("RR_DETECT_LATERAL")
      corner = True
    if corner:
      # ▼▼▼ [수정] 측방 차량 속도 계산을 통한 BSD 사각지대(블라인드 스팟) 완벽 방어 ▼▼▼
      if not hasattr(self, 'bs_timer'):
        self.corner_hist = {"LF": deque(maxlen=15), "RF": deque(maxlen=15), "LR": deque(maxlen=15), "RR": deque(maxlen=15)}
        self.bs_timer = {"L": 0, "R": 0}
        self.last_approach_time = {"L": 0.0, "R": 0.0}
      
      def calc_speed_and_time(hist, current_dist):
        if current_dist > 0.1:
          hist.append(current_dist)
          if len(hist) >= 5:
            # v = (현재 거리 - 과거 거리) / 시간. 
            # 차가 다가오고 있다면 거리가 좁혀지므로 v는 음수(-)가 됨
            v = (hist[-1] - hist[0]) / (len(hist) * DT_CTRL)
            if v < -0.5:
              # 사각지대 구간(약 4.5m)을 해당 속도로 통과하는 데 걸리는 시간 예상 (최대 3초)
              return min(4.5 / abs(v), 3.0) 
            elif abs(v) <= 0.5:
              # 속도 차이 없이 나란히 달리고 있다면 기본 2.5초 유지
              return 2.5
        else:
          hist.clear()
        return 0.0

      # 1. 4개의 코너 레이더 각각의 접근 속도를 계산하여 사각지대 예상 통과 시간 산출
      time_lf = calc_speed_and_time(self.corner_hist["LF"], ret.leftLongDist)
      time_rf = calc_speed_and_time(self.corner_hist["RF"], ret.rightLongDist)
      time_lr = calc_speed_and_time(self.corner_hist["LR"], self.lr_distance)
      time_rr = calc_speed_and_time(self.corner_hist["RR"], self.rr_distance)

      # 2. 현재 레이더 시야에 차량이 존재하는지 확인
      left_front_seen = 0 < ret.leftLongDist < 3.0
      left_rear_seen = 0 < self.lr_distance < 5.0
      right_front_seen = 0 < ret.rightLongDist < 3.0
      right_rear_seen = 0 < self.rr_distance < 5.0

      # --- 3. 좌측 사각지대(BSD) 로직 ---
      if left_front_seen or left_rear_seen:
        # 차가 레이더에 보일 때는 끄트머리에서 사라질 때를 대비해 통과 시간을 계속 저장해둠
        self.last_approach_time["L"] = max(time_lf, time_lr, 0.5) 
        self.bs_timer["L"] = 0
        ret.leftBlindspot = True
      else:
        # 차가 레이더에서 막 사라진 순간 (사각지대 진입) -> 저장해둔 시간만큼 타이머 장전!
        if self.last_approach_time["L"] > 0:
          self.bs_timer["L"] = int(self.last_approach_time["L"] / DT_CTRL)
          self.last_approach_time["L"] = 0.0 
          
        # 예상 시간이 끝날 때까지 가상으로 BSD 점등을 꽉 쥐고 유지함 (차선변경 방어)
        if self.bs_timer["L"] > 0:
          self.bs_timer["L"] -= 1
          ret.leftBlindspot = True

      # --- 4. 우측 사각지대(BSD) 로직 ---
      if right_front_seen or right_rear_seen:
        self.last_approach_time["R"] = max(time_rf, time_rr, 0.5)
        self.bs_timer["R"] = 0
        ret.rightBlindspot = True
      else:
        if self.last_approach_time["R"] > 0:
          self.bs_timer["R"] = int(self.last_approach_time["R"] / DT_CTRL)
          self.last_approach_time["R"] = 0.0
          
        if self.bs_timer["R"] > 0:
          self.bs_timer["R"] -= 1
          ret.rightBlindspot = True
      # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲
        
    if self.hda_info_4a3 is not None:
      speedLimit = self.hda_info_4a3["SPEED_LIMIT"]
      if not self.is_metric:
        speedLimit *= CV.MPH_TO_KPH
      ret.speedLimit = speedLimit if speedLimit < 255 else 0
      
      # ▼▼▼ [추가된 핵심 로직] 4BE가 섞이기 전에 순수 4A3 신호만 밖으로 빼냅니다! ▼▼▼
      ret.navSpeedLimit = ret.speedLimit
      ret.mapSource = int(self.hda_info_4a3.get("MapSource", 0))  # 💡 에러 방지 안전장치!
      ret.navLinkClass = int(self.hda_info_4a3.get("LinkClass", 0)) # 💡 IC/JC 판별용 링크 클래스 추가!
      ret.navTollExist = int(self.hda_info_4a3.get("TollExist", 0)) # 💡 [추가] 톨게이트 판별용 추가!
      ret.navFrwinfo = int(self.hda_info_4a3.get("Frwinfo", 0))     # 💡 [추가] Frwinfo 신호 전달!
      # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

      if int(self.hda_info_4a3.get("MapSource", 0)) == 2:         # 💡 에러 방지 안전장치!
        speed_limit_cam = True

      if self.time_zone == "UTC":
        country_code = int(self.hda_info_4a3.get("CountryCode", 0)) # 💡 에러 방지 안전장치!
        self.time_zone = ZoneInfo(NUMERIC_TO_TZ.get(country_code, "UTC"))

    ret.gearStep = cp.vl["GEAR"]["GEAR_STEP"] if self.GEAR else 0
    if 1 <= ret.gearStep <= 8 and ret.gearShifter == GearShifter.unknown:
      ret.gearShifter = GearShifter.drive
    ret.gearStep = cp.vl["GEAR_ALT"]["GEAR_STEP"] if self.GEAR_ALT else ret.gearStep

    lane_info = self.cam_0x2a4 if self.cam_0x2a4 is not None else self.cam_0x362

    if lane_info is not None:
      left_lane_prob = lane_info["LEFT_LANE_PROB"]
      right_lane_prob = lane_info["RIGHT_LANE_PROB"]
      left_lane_type = lane_info["LEFT_LANE_TYPE"] # 0: dashed, 1: solid, 2: undecided, 3: road edge, 4: DLM Inner Solid, 5: DLM InnerDashed, 6:DLM Inner Undecided, 7: Botts Dots, 8: Barrier
      right_lane_type = lane_info["RIGHT_LANE_TYPE"]
      left_lane_color = lane_info["LEFT_LANE_COLOR"]
      right_lane_color = lane_info["RIGHT_LANE_COLOR"]
      left_lane_info = left_lane_color * 10 + left_lane_type
      right_lane_info = right_lane_color * 10 + right_lane_type
      ret.leftLaneLine = left_lane_info
      ret.rightLaneLine = right_lane_info

    # Manual Speed Limit Assist is a feature that replaces non-adaptive cruise control on EV CAN FD platforms.
    # It limits the vehicle speed, overridable by pressing the accelerator past a certain point.
    # The car will brake, but does not respect positive acceleration commands in this mode
    # TODO: find this message on ICE & HYBRID cars + cruise control signals (if exists)
    if self.CP.flags & HyundaiFlags.EV:
      if self.manual_speed_limit_assist is not None:
        #ret.cruiseState.nonAdaptive = cp.vl["MANUAL_SPEED_LIMIT_ASSIST"]["MSLA_ENABLED"] == 1
        ret.cruiseState.nonAdaptive = self.manual_speed_limit_assist["MSLA_ENABLED"] == 1

    if self.LOCAL_TIME and self.time_zone != "UTC":
      lt = cp.vl["LOCAL_TIME"]
      y, m, d, H, M, S = int(lt["YEAR"]) + 2000, int(lt["MONTH"]), int(lt["DATE"]), int(lt["HOURS"]), int(lt["MINUTES"]), int(lt["SECONDS"])
      try:
        dt_local = datetime(y, m, d, H, M, S, tzinfo=self.time_zone)
        ret.datetime = int(dt_local.timestamp() * 1000)
      except:
        #print(f"Error parsing local time: {y}-{m}-{d} {H}:{M}:{S} in {self.time_zone}")
        pass

    prev_cruise_buttons = self.cruise_buttons[-1]
    #self.cruise_buttons.extend(cp.vl_all[self.cruise_btns_msg_canfd]["CRUISE_BUTTONS"])
    #carrot {{

    if self.cruise_buttons_alt2 is not None:
      if int(self.cruise_buttons_alt2.get("LFA_BTN", 0)) == 1:
        cruise_button = [Buttons.LFA_BUTTON]
      else:
        v = int(self.cruise_buttons_alt2.get("CRUISE_BUTTONS", 0))
        cruise_button = [v if v < 5 else Buttons.NONE]
    elif cp.vl[self.cruise_btns_msg_canfd]["LFA_BTN"]:
      cruise_button = [Buttons.LFA_BUTTON]
    else:
      cruise_button = cp.vl_all[self.cruise_btns_msg_canfd]["CRUISE_BUTTONS"]

    self.cruise_buttons.extend(cruise_button)
    # }} carrot


    #if self.cruise_btns_msg_canfd in cp.vl:
    #  self.cruise_buttons_msg = copy.copy(cp.vl[self.cruise_btns_msg_canfd])
    """
    if self.cruise_btns_msg_canfd in cp.vl: #carrot
      if not cp.vl[self.cruise_btns_msg_canfd]["CRUISE_BUTTONS"]:
        pass
        #print("empty cruise btns...")
      else:
        self.cruise_buttons_msg = copy.copy(cp.vl[self.cruise_btns_msg_canfd])
     """
    prev_main_buttons = self.main_buttons[-1]
    #self.cruise_buttons.extend(cp.vl_all[self.cruise_btns_msg_canfd]["CRUISE_BUTTONS"])
    if self.cruise_buttons_alt2 is not None:
      self.main_buttons.extend([1 if int(self.cruise_buttons_alt2.get("CRUISE_BUTTONS", 0)) == 8 else 0])
    else:
      self.main_buttons.extend(cp.vl_all[self.cruise_btns_msg_canfd]["ADAPTIVE_CRUISE_MAIN_BTN"])
    if self.main_buttons[-1] != prev_main_buttons and not self.main_buttons[-1]: # and self.CP.openpilotLongitudinalControl: #carrot
      self.main_enabled = not self.main_enabled
      print("main_enabled = {}".format(self.main_enabled))
    self.buttons_counter = cp.vl[self.cruise_btns_msg_canfd]["COUNTER"]
    ret.accFaulted = cp.vl["TCS"]["ACCEnable"] != 0  # 0 ACC CONTROL ENABLED, 1-3 ACC CONTROL DISABLED

    speed_conv = CV.KPH_TO_MS # if self.is_metric else CV.MPH_TO_MS
    cluSpeed = cp.vl["CRUISE_BUTTONS_ALT"]["CLU_SPEED"]
    ret.vEgoCluster = cluSpeed  * speed_conv # MPH단위에서도 KPH로 나오는듯..
    vEgoClu, aEgoClu = self.update_clu_speed_kf(ret.vEgoCluster)
    ret.vCluRatio = (ret.vEgo / vEgoClu) if (vEgoClu > 3. and ret.vEgo > 3.) else 1.0

    # ▼▼▼ [수정] 토글 상태 갱신 및 4A3/4BE 우선순위 로직 실행 ▼▼▼
    self.frame_for_params += 1
    if self.frame_for_params % 100 == 0:
      self.vehicleNaviCanControl = Params().get_bool("VehicleNaviCanControl")

    # 기존 코드
    cam_limit, cam_dist = self._update_vehicle_navi_events(cp)
    nav_toll_exist = getattr(ret, 'navTollExist', 0)
    
    # ▼▼▼ [추가] 링크 클래스 확인 ▼▼▼
    nav_link_class = getattr(ret, 'navLinkClass', 0)
    is_ic_jc = (nav_link_class in [2, 3])

    # ▼▼▼ [수정] 카메라, 톨게이트, 4A3없음 + 우측깜빡이(20초) 또는 IC/JC 진입 시 4BE를 통과시킵니다! ▼▼▼
    if cam_limit > 0 and (cam_dist > 0 or nav_toll_exist != 0 or ret.speedLimit == 0 or self.right_blinker_timer > 0 or is_ic_jc):
      ret.speedLimit = cam_limit
      if cam_dist > 0:
        speed_limit_cam = True
        self.speedLimitDistance = self.totalDistance + cam_dist
      else:
        speed_limit_cam = False

    self.update_speed_limit(ret, speed_limit_cam)

    paddle_button = self.paddle_button_prev
    if self.cruise_btns_msg_canfd == "CRUISE_BUTTONS":
      paddle_button = 1 if cp.vl["CRUISE_BUTTONS"]["LEFT_PADDLE"] == 1 else 2 if cp.vl["CRUISE_BUTTONS"]["RIGHT_PADDLE"] == 1 else 0
    elif self.gear_msg_canfd == "GEAR":
      paddle_button = 1 if cp.vl["GEAR"]["LEFT_PADDLE"] == 1 else 2 if cp.vl["GEAR"]["RIGHT_PADDLE"] == 1 else 0

    ret.buttonEvents = [*create_button_events(self.cruise_buttons[-1], prev_cruise_buttons, BUTTONS_DICT),
                        *create_button_events(paddle_button, self.paddle_button_prev, {1: ButtonType.paddleLeft, 2: ButtonType.paddleRight}),
                        *create_button_events(self.main_buttons[-1], prev_main_buttons, {1: ButtonType.mainCruise})]

    self.paddle_button_prev = paddle_button
    return ret

  def get_can_parsers_canfd(self, CP):
    msgs = []
    # ▼▼▼ [수정] 4B9, 4BE 등록 ▼▼▼
    msgs = [("NEW_MSG_4B9", math.nan), ("NEW_MSG_4BE", math.nan)]
    # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲
    if not (CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS):
      # TODO: this can be removed once we add dynamic support to vl_all
      msgs += [
        ("CRUISE_BUTTONS", 50)
      ]
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], msgs, CanBus(CP).ECAN),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus(CP).CAM),
      Bus.alt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus(CP).ACAN),
    }

  def get_can_parsers(self, CP):
    if CP.flags & HyundaiFlags.CANFD:
      return self.get_can_parsers_canfd(CP)

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
