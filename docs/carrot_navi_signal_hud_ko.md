# 당근내비 신호등 HUD 표시 (읽기 전용 시험판)

대상: `ms-xy-radar3` 브랜치. 분할화면 관련 코드는 포함하지 않습니다.

## 데이터 경로

당근내비 1.30 시험판 신호등 원본 JSON
→ 기존 CarrotNavi V2 WebSocket의 `traffic_signal` 스트림, 또는 기존 `route`·`guidance_current` 등에 삽입된 `traffic_signal` 객체
→ `selfdrive/carrot/navi_signal_hud.py` (유효성 검사)
→ `carrot_navi_cereal.py`의 `signalHud`
→ `selfdrive/ui/carrot.cc`의 좌측 하단 속도 HUD 위 정보 박스.

기존 `trafficSignal` 필드는 주행 제어에서도 참조될 수 있기 때문에 **전혀 변경하지 않았습니다.**
새로운 `signalHud`는 UI 전용, 차량 제어 연결 금지입니다.

## 표시 규칙

- HUD 박스의 왼쪽 정렬 위치/폭(475px)을 유지하고, 기존 박스 바로 위 약 172px 영역에만 배치.
- 현재 신호 색상(빨강/노랑/초록 및 점멸), 진행 방향(직진/좌/우/유턴), 원본의 남은 초를 표시.
- 카메라로 검출한 신호와는 별도 데이터. 원본 `direction_match_verified=false`인 경우 **`교차로·차로 매칭 미확인`**을 항상 표시.
- 실제 색상은 내비 서버의 참고 데이터이므로 차량 제어에 사용하지 않음.
- 원본 시간이 없거나 범위가 벗어난 경우 카운트다운 대신 `시간 미제공` 표시. 로컬에서 매초 임의로 감소시키지 않음.
- 최대 두 항목만 표시. 세 항목 이상이거나 원본 방향값을 모르면 임의 선택 없이 숨김.
- 오픈파일럿 통신 연결 해제, 항목 만료/부재, 데이터 2.5초 초과, 불명 상태는 즉시 숨김.

## 현재 제한

**당근내비 APK에서 실시간 신호등 패킷이 실제로 WebSocket 스트림에 도착하는지는 실차 로그로 아직 검증되지 않았습니다.**
기존 APK 시험판의 Java 코드에서 `traffic_signal` 객체를 추가하더라도, 실제 WebSocket 스트림의 `value`에 포함되지 않으면 패널은 나타나지 않습니다.
또한 `SignalGuideInfo`의 `ONE`, `FIVE` 등 미해석 방향은 표시하지 않습니다.

`SignalHud`는 `cereal/custom.capnp`에 기존 필드들과 독립적으로 추가됩니다. 빌드 과정에서 Cereal 스키마 생성이 필요합니다.

## 확인 명령

```bash
python -m pytest -q selfdrive/carrot/tests/test_navi_signal_hud.py
g++ -std=c++17 -Wall -Wextra -pedantic -I. \
  selfdrive/ui/tests/test_navi_signal_hud_standalone.cc -o /tmp/test_navi_signal_hud
/tmp/test_navi_signal_hud
```

차량에 설치하기 전에 기존 네비/ATC/크루즈 코드와 함께 전체 UI 컴파일 및 기존 테스트를 통과해야 합니다.
실차에선 정차 후 CarrotNavi /latest의 `traffic_signal` 및 `route` 데이터에 원본 `signals`가 들어오는지 먼저 확인합니다.
사용자가 주행 중 조작하거나 신호등 정보만으로 출발/감속하지 않도록 해야 합니다.

## 되돌리기

별도 브랜치의 PR을 병합하지 않으면 기존 `ms-xy-radar3` 동작에는 영향이 없습니다.
