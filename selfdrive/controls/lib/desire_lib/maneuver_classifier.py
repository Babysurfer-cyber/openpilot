from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from .constants import BLINKER_LEFT, BLINKER_RIGHT

def classify_maneuver_type(blinker_state: int,
                           carstate,
                           side,                 # SideState
                           turn_desire_state: bool,
                           atc_type: str,
                           old_type: str):
  if blinker_state == 0:
    return "none"

  v_kph = carstate.vEgo * CV.MS_TO_KPH
  accel = carstate.aEgo

  # 💡 [추가] 깜빡이 레버가 끝까지 제쳐진 상태(2)인지 판별하는 변수
  lever_2 = getattr(carstate, "blinkerLever", 0) == 2

  score_turn = 0
  
  # 1. 저속 주행 조건 + 레버 2번 제낌
  if v_kph < 30.0 and lever_2:
    score_turn += 1
  # 2. 저속 감속 조건 + 레버 2번 제낌
  elif v_kph < 40.0 and accel < -1.0 and lever_2:
    score_turn += 1

  # 3. 차선 및 에지가 없는 저속 조건 + 레버 2번 제낌
  if v_kph < 40.0 and (not side.lane_available) and (not side.edge_available) and lever_2:
    score_turn += 1

  # 4. 차선 존재 카운트 조건 + 레버 2번 제낌
  if v_kph < 40.0 and side.lane_exist_count.counter < int(0.5 / DT_MDL) and lever_2:
    score_turn += 1

  # 5. 모델의 턴 의도 상태 + 레버 2번 제낌
  if turn_desire_state and lever_2:
    score_turn += 1

  # ATC(내비게이션 등 시스템 제어) 타입에 따른 스코어 보정
  if atc_type in ("turn left", "turn right"):
    score_turn += 2
  elif atc_type in ("fork left", "fork right", "atc left", "atc right"):
    score_turn -= 2

  edge_far = side.dist_to_edge_far > 4.0

  if score_turn >= 2:
    if edge_far:
      return "turn"
    return old_type
  return "lane_change"
