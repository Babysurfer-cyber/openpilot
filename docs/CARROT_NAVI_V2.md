# 당근내비 v2 연동

`ajouatom/openpilot`의 `carrot-wip` 커밋
[`5c970aa51678`](https://github.com/ajouatom/openpilot/tree/5c970aa5167811a9d90adbc36e15dcdef54b1ce4/openpilot/selfdrive/carrot)을
기준으로 최신 당근내비의 JSON 통신을 C3-wip 기반 구조에 이식했습니다.
프로토콜 기준은 해당 커밋의 `carrot_navi_api.md`입니다.

## 연결

- 스마트폰과 콤마를 같은 네트워크에 연결합니다.
- `carrot_navi` 프로세스가 UDP 7705로 각 네트워크 인터페이스의 주소를 알립니다.
- 앱은 TCP 7714의 `/api/navi/ws/v2/control/{version}`에 연결한 뒤 JSON 스트림을 엽니다.
- 앱이 요구하는 전체 28개 항목을 manifest에 포함합니다. JSON 13개만 활성화하고 이미지·지도 영상 15개는 비활성화합니다.
- 별도의 연결 옵션은 추가하지 않았습니다. 업데이트 후 재부팅하면 새 프로세스가 함께 실행됩니다.

ATC는 기존 설정을 따릅니다. `AutoTurnControl`은 0: 끔, 1: 차선변경,
2: 차선변경과 속도 제어, 3: 속도 제어입니다. 이식 과정에서 저장된 설정을 변경하지 않습니다.

## 데이터 흐름

`carrot_navi` → `carrotNavi` 메시지 → `CarrotServ`의 기존 TBT/SDI 처리
→ `carrotMan` → 기존 `DesireHelper`와 크루즈 처리 순서입니다.
현재·다음 안내, 제한속도·단속·구간단속, GPS, 경로 및 도로 분류를 연결합니다.
교통신호 JSON은 수신·기록하며 기존 영상 신호 인식의 출발 판정을 변경하지 않습니다.

기존 TBT/SDI 변환과 ATC 판단, 수동 턴 감속, 원터치 차선변경,
코너레이더, 오토모드 및 HDA 부팅 처리의 핵심 코드는 보존합니다.
구형 UDP/HTTP 입력도 유지하되, v2 연결 중에는 구형 안내가 새 안내를 덮어쓰지 않습니다.

## 만료와 경로 선택

- JSON은 변경 시 즉시, 같은 값도 1초마다 보내도록 요청합니다.
- 항목별 실제 수신 시각을 기준으로 3초가 지나면 해당 항목을 해제합니다.
- 프로세스 heartbeat는 최대 0.5초 간격이며, 소비자는 1초 넘게 수신하지 못한 서비스를 사용하지 않습니다.
- 제어 소켓 종료 시 세션과 항목을 비웁니다. 새 세션은 이전 sequence와 분리합니다.
- 안내 종료·경로 이탈·안내 만료 시 ATC 요청과 해당 경로를 해제합니다.
- v2 안내 중에는 별도 Mapbox 경로를 섞지 않습니다. Mapbox 경로는 따로 보관해 외부 내비 종료 후 다시 선택합니다.
- GPS 수신 시각과 주행으로 줄어든 안내 거리를 단순 heartbeat로 초기화하지 않습니다.

## 검증 범위

`test_carrot_navi_protocol.py`는 실제 로컬 WebSocket 연결과 네이티브
Cap'n Proto 메시지 변환을 검사합니다. `test_carrot_navi_atc.py`는
기존 `CarrotServ.update_navi()`의 ATC·감속 출력과 만료 처리를,
`test_carrot_navi_route.py`는 경로 선택과 재연결을 검사합니다.
실제 스마트폰 앱 및 차량과의 연동은 별도 실기 확인이 필요합니다.
