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

- **좌측 하단 기존 HUD 창 바로 위**에 좌측 정렬된 475×68px 검정 반투명 알약 형태. 간격 10px.
- 왼쪽부터 **빨강 원 → 주황 원 → 좌회전 초록 화살표 → 직진 초록 원 → 오른쪽 남은 초**.
- 각 등은 꺼지면 어두운 회색. 활성 신호는 밝은 색과 약한 후광. 원본 점멸 신호는 해당 표시만 점멸.
- 주황은 데이터의 YELLOW(노랑) 상태에 대응한다.
- LEFT 방향 GREEN은 3번 좌회전 화살표, STRAIGHT 방향 GREEN은 4번 직진 원.
- 방향별 원본 신호가 여러 개이면 모두 같은 group_index(교차로 묶음)인지 확인한다. 직진·좌회전 방향 중복이나 다른 교차로 묶음 등 애매한 경우 표시하지 않는다.
- 잔여시간은 네이버지도 SDK와 같이 일반 초록 → 노랑 → 빨강 순으로 하나를 선택한다. 동순위이면 수신 목록의 첫 신호를 유지하며, 점멸은 카운트다운 후보에서 제외한다.
- 선택된 신호의 유효한 잔여시간이 **6초 이상**일 때만 해당 색상으로 숫자를 표시한다. 짧거나 누락된 경우 `--`를 표시하고 다른 신호의 시간으로 대체하지 않는다.
- 우회전의 일반 신호와 유턴 초록도 SDK의 시간 선택 후보에 포함하지만, 기존 네 개 점등 위치는 그대로 유지한다. 이 우선순위는 현재 차량 차로와의 일치 검증을 의미하지 않는다.
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


## 기존 APK 무수정 수신 지원 (이번 PR)

기존 당근내비 APK가 이미 전송하는 신호 이벤트를 **콤마에서만** 처리합니다.
APK 재빌드, Java/Smali 수정, V2 프로토콜 확장은 필요하지 않습니다.

- **기존 TCP 7712 / HTTP 7713:** `carrot_man.py`가 수신하는 `sinf` 및 `ssinf`를 기존 내비 로직을 유지한 채, `navi_signal_legacy.py`에서 표시 전용 관측치로 변환합니다. `ssinf`가 동시에 있으면 상세 방향 정보를 우선합니다.
- **기존 WebSocket V2 7714:** 원래 `traffic_signal.value.lights`의 빨강·직진·좌회전 활성 신호와 남은 시간을 읽습니다. 방향별 `movements`에 인식 가능한 상태가 있으면 우선합니다. 새로운 `schema_version/source/signals` 래핑을 APK에 강요하지 않습니다.
- **프로세스 간 전달:** 기존 `carrot_man`과 별도 `carrot_navi` 게시 프로세스 사이에서 `/dev/shm/carrot_navi_legacy_signal_hud.json`만 원자적 교체 방식으로 사용합니다. **수신 시각을 새로 갱신하지 않는** 2.5초 TTL로, 신호가 끊기면 HUD에 남지 않습니다.
- **원본보다 상세한 정보를 꾸며내지 않습니다.** `sinf.redLightOn`은 전역 빨간불 표시이며, 직진 차로 신호임이 검증된 값은 아닙니다. 빨강+직진초록이 동시에 참이면 모호하므로 숨깁니다. 노랑은 `ssinf` 또는 V2에서 명시적으로 보내는 경우에만 표시하며, `sinf`의 누락된 노랑을 추측하지 않습니다.
- **제어 분리:** `signalHud`만 갱신합니다. `carrotMan.trafficState`, `DETECT`, `TrafficLightDetectMode`, 가감속, ATC, 크루즈는 변경하지 않습니다. V2 연결이 없어도 signalHud는 보일 수 있으나, 기존 `carrotNavi.connected`는 거짓 그대로 유지합니다.

### 검증 방법

```bash
python -m pytest -q selfdrive/carrot/tests/test_navi_signal_legacy.py selfdrive/carrot/tests/test_navi_signal_hud.py
```

1. 설치된 **기존 APK를 그대로** 실행하고 정차 상태에서 경로를 시작합니다.
2. `carrot_man` 수신 로그 또는 기존 내비 진단에서 `sinf`, `ssinf` 혹은 V2의 `traffic_signal`이 **실제로** 도착하는지 확인합니다. APK 버전에 따라 신호 메시지가 전혀 없을 수 있습니다.
3. 수신되면 왼쪽 하단 HUD 위에 빨강/노랑/좌회전/직진 표시가 나타나는지 확인하고, 방향별 정보·거리·남은 시간의 정확성을 교차 검증합니다.
4. 앱 연결을 끊거나 해당 신호 전송을 멈춘 뒤 2.5초 내 사라지는지 확인합니다. 테스트는 차량 정지 중 실시합니다.

**중요:** 브랜치 변경만으로 APK가 전송하지 않는 데이터를 새로 만들어낼 수는 없습니다.
기존 APK가 해당 이벤트를 이미 내보내는지 확인해야 합니다.
