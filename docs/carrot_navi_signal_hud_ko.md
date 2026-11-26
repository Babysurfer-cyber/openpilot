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

## 표시 규칙 (2026-11-26 변경)

- **좌측 하단 기존 HUD 창 바로 위**에 좌측 정렬된 475×68px 검정 반투명 알약 형태. 간격 10px.
- 왼쪽부터 **빨강 원 → 주황 원 → 좌회전 초록 화살표 → 직진 초록 원 → 오른쪽 남은 초**.
- 각 등은 꺼지면 어두운 회색. 활성 신호는 밝은 색과 약한 후광. 원본 점멸 신호는 해당 표시만 점멸.
- 주황은 데이터의 YELLOW(노랑) 상태에 대응한다.
- LEFT 방향 GREEN은 3번 좌회전 화살표, STRAIGHT 방향 GREEN은 4번 직진 원.
- 방향별 원본 신호가 여러 개이면 모두 같은 group_index(교차로 묶음)인지 확인한다. 같은 방향의 복수 신호, 다른 교차로 묶음 등 애매한 경우 표시하지 않는다.
- 방향별 남은 시간이 서로 다르거나 어느 신호의 남은 시간이 누락되면 오른쪽은 `--`. 하나의 시간을 임의로 대표값으로 선택하지 않는다.
- 남은 초는 실시간 원본 값을 그대로 표시하며 임의 카운트다운을 하지 않는다.
- 통신 중단·부재·2.5초 초과 시 즉시 숨김. 출처의 신호등이 실제 주행 차로와 일치하는지는 아직 검증되지 않았다.
- **시각 정보 전용.** 기존 `trafficSignal`과 독립된 `signalHud` 필드이며 크루즈·ATC·가감속 제어에는 절대 연결하지 않는다.
- 표시 코드는 `selfdrive/ui/carrot.cc`, 판별 로직은 `selfdrive/ui/navi_signal_hud.h`, 수신 검사는 `selfdrive/carrot/navi_signal_hud.py`.

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
