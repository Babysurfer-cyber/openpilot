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

  score_turn = 0

  # ▼▼▼ [핵심] carrot-wip의 blinkerLever 값 확인 (1: Tap, 2: Latched) ▼▼▼
  blinker_lever = getattr(carstate, 'blinkerLever', 2)

  # 완전히 제쳤을 때(blinker_lever == 2)만 교차로 회전(Turn) 가중치 부여
  if blinker_lever == 2:
    if v_kph < 50.0:
      score_turn += 1
    elif v_kph < 60.0 and accel < -1.0:
      score_turn += 1

    if v_kph < 60.0 and (not side.lane_available) and (not side.edge_available):
      score_turn += 1

    if v_kph < 60.0 and side.lane_exist_count.counter < int(0.5 / DT_MDL):
      score_turn += 1

    if turn_desire_state:
      score_turn += 1

  if atc_type in ("turn left", "turn right"):
    score_turn += 2
  elif atc_type in ("fork left", "fork right", "atc left", "atc right"):
    score_turn -= 2

  edge_far = side.dist_to_edge_far > 4.0

  if score_turn >= 2:
    if edge_far:
      return "turn"
      
    # ▼ 차선이 다시 보이면(턴 종료), 깜빡이가 남아있어도 턴/차선변경 모두 즉시 강제 종료!
    if side.lane_available:
      return "none" 
      
    return old_type
  return "lane_change"
