# EV6 SPAS2 제어기 개발 초안 — NO VEHICLE TX

## 목표와 현재 범위

원작자 [ajouatom/openpilot `joongyu01/carrot-wip-dev`](https://github.com/ajouatom/openpilot/tree/joongyu01/carrot-wip-dev)의
`opendbc_repo/opendbc/car/hyundai/spas.py`,
`carcontroller.py`, `hyundaicanfd.py`,
`safety/safety/safety_hyundai_spas.h`를 비교해 **독립 ECU 제어권 증거를
필요로 하는 상태 관리 코어**를 만들었다.

- 새 모듈: `ev6_spas_control.py`
- 테스트: `test_ev6_spas_control.py`
- **현재 상태: 소프트웨어 계획기만 구현. 차량 CAN 송신/UDS 요청/ECU 제어 없음.**
- `WOULD_*` 출력은 가상 *의도 이벤트* 문자열이다. CANPacket, `CanData`,
  `can_sends`, panda TX 허용 목록, 원격 차량 명령으로 연결되지 않는다.
- 주행 브랜치 `ms-xy-radar3`, `interface.py`, `carcontroller.py`,
  `hyundaicanfd.py` 또는 실제 panda 안전 정책에 어떤 변경도 하지 않는다.

## 원작에서 가져온 검증 구조

| 항목 | 이 초안의 적용 |
| --- | --- |
| 원본 SPAS1 0x165/24B, SPAS2 0x16A/32B | 정확한 길이, CRC16/XMODEM 및 증가 카운터 |
| ECU 진단 응답 0x7B9 | 적절한 버스, 형식, 순서, 400ms 이내의 긍정 응답만 수락 |
| Extended session → post-session stock → disable ACK | 요청 후에도 원본 두 스트림이 최근 들어왔는지 확인 |
| ECU stream silence | 진단 ACK 뒤 두 스트림이 실제로 조용해야만 **모의 준비 상태** |
| 순정 ECU 복귀 | 준비 상태 중 순정 SPAS가 다시 보이면 즉시 복구 상태로 전환 |
| 안전 조건 | P, 브레이크, 정차, openpilot 제어 비활성, 레버 중립, 버스 정상, **독립 버스 모니터** |
| 복구/잠금 | communication enable, default session, 두 순정 스트림 복귀 증거 없으면 locked |
| 요청 | 좌/우 단발, 요청 ID 재사용 차단, 중간 취소 및 350ms 모의 만료 |

원작 `spas.py`의 장점인 순수 상태 관리 방식과 응답/카운터 검증을
재사용했지만, 다음은 **의도적으로 이식하지 않았다**.

- ECU가 응답하지 않을 때 SPAS2를 송신하는 fallback
- stock 프레임과 동일한 CAN ID를 중복 송신하는 piggyback/direct 전송
- `0x7B1` 명령 생성 또는 자동 전송, ECU disable
- 스티어링/주차 지원 제어, hazard 및 주행 중 방향지시등 제어
- CAN 신호 값을 근거 없이 변조해 전송하는 템플릿

원작 일부 로그는 기존 ECU 응답이 없고 동일 ID 중복 송신에서 bus-off를
기록했다. SPAS2의 `BLINKER_CONTROL=3/4`와 순정 램프 점등의 상관은
이미 관측했지만, 외부 송신 성공이나 안전한 ECU 인계는 입증되지 않았다.

## 검증 커맨드

~~~bash
cd opendbc_repo
python -m unittest discover -s opendbc/car/hyundai -p test_ev6_spas_control.py -v
~~~

현재 GitHub에 복사한 소스와 로컬 실행본 FNV32/문자 수가 일치했다.
로컬 Python 3.13.5에서 **16/16 synthetic tests PASS**:
- 코어: 9416자, `0x862f1e31`
- 테스트: 5779자, `0x59510cb6`

이는 **실제 ECU 테스트, panda libsafety C 빌드, CI, 차량 점등 확인이 아니다**.

## 다음 진행에 필요한 독립 검증

1. 독립 ECU/CAN-FD 벤치에서 `0x7B9` 긍정 응답과 `0x165/0x16A`
   원본 송신의 완전한 정지 및 복구를 측정한다.
2. ECU 손실/응답 없음, CRC 실패, 역순 응답, 버스 오류 증가, 재부팅,
   복구 실패 등의 실패 모드를 검증한다.
3. 별도 설계 검토로 주차 센서·서라운드뷰·주차보조 등 다른 기능의
   부작용과 fail-safe 수단을 확인한다.
4. 이 조건이 통과되기 전에는 운행 차량에 UDS / SPAS 제어 송신기를
   연결하지 않는다.

**현재 NO-GO: PR Draft 유지, 병합/차량 설치 금지.**


## 2026-11-25 보강: 제어권/복구 오판 결함 수정 및 회귀검증

기존 16개 정상 동작 모의시험에 실패 경로 8개를 추가해 **수정 전
8개 모두 FAIL**하는 결함을 확인했다. 수정 후 24/24, 추가 복구
강화 테스트 4개를 더해 **최신 28/28 Python unittest PASS**.
현재 GitHub 브랜치에 적용한 파일을 로컬 실행본과 문자 수/FNV32로
대조했다(코어 12,370자 / `0x89a120a2`, 테스트 10,363자 /
`0xca6c28b9`). 소스 문법 검사와 금지된 CAN 송신 API 명칭의
AST 감사도 통과했다. **GitHub Actions·panda C 전체 빌드와 실 ECU
검증은 아직 수행했다고 주장하지 않는다.**

추가·수정된 안전 조건:

- 세션/disable/복구 ACK는 **현 단계의 시한 내** 받아야만 유효.
  과거 ACK를 한참 뒤 처리해도 진행되던 버그 수정
- 복구 명령에 대한 음성/잘못된 응답과 복구 중 안전 조건 상실은
  `locked`로 고정; 이후 좋은 응답으로 성공 처리 불가
- 복구에 사용할 순정 SPAS1·SPAS2는 default-session 응답 뒤에
  **각각 새로 수신된 10개 연속 유효 프레임**을 요구한다.
  PR #45 독립 벤치 판정기의 연속 10프레임 요구와 정렬
- 복구 중 CRC 실패 또는 counter 불연속 발견 시 잠금
- 순정 SPAS2 방향 명령이 이미 활성화된 상태에서 별도 좌/우
  요청 거부; 중간에 활성화되어도 준비 상태 취소
- 입력 객체의 `true` 문자열 같은 truthy 비boolean 값을 거부
- quiet 상태의 장시간 업데이트 단절을 원본 ECU의 정지로 오인하지
  않고 `monitor_continuity_lost`로 복구 전환

**한계:** 독립 CAN-FD 모니터의 건강 상태는 아직 실제 측정값이 아니라
`SafetyInputs.independent_bus_monitor`라는 호출자의 입력으로 들어온다.
로그가 단절된 사이의 안전성, 건강 상태의 장비 독립성 및 ECU 출처는
이 시뮬레이터만으로 인증할 수 없다. 장비별 프로토콜과 장애 처리는
기존 PR #45의 독립 검증기와 분리하여 실제 격리 벤치에서 평가해야
한다. 여기에는 CAN/UDS TX 함수나 실차 연결 코드가 전혀 없다.

**계속 Draft / NO-GO:** 실차 운행 브랜치 병합, 0x7B1 진단 송신,
0x165/0x16A 중복 송신 또는 주행 중 자동 방향지시등 시험은 승인되지 않았다.
