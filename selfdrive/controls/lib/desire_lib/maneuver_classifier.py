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
  
  # ▼▼▼ [수정] 완전 체결(blinkerLever == 2)일 때만 Turn 점수 부여 ▼▼▼
  if v_kph < 30.0 and carstate.blinkerLever == 2:
    score_turn += 1
  elif v_kph < 40.0 and accel < -1.0 and carstate.blinkerLever == 2:
    score_turn += 1

  # 차선 및 edge 유무에 따른 turn 판단
  if v_kph < 40.0 and (not side.lane_available) and (not side.edge_available) and carstate.blinkerLever == 2:
    score_turn += 1

  # 맨 끝 차선 확인
  if v_kph < 40.0 and side.lane_exist_count.counter < int(0.5 / DT_MDL) and carstate.blinkerLever == 2:
    score_turn += 1

  # 모델의 회전 예측
  if turn_desire_state and carstate.blinkerLever == 2:
    score_turn += 1
  # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

  # ATC (내비게이션 자동 턴/차선변경)는 운전자 레버 조작과 무관하게 독립적으로 판단
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
