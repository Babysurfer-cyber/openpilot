# 2022 EV6 SPAS2 원작자 코드 대조 — 수신 전용 포팅 검토

**원작:** [ajouatom/openpilot, joongyu01/carrot-wip-dev](https://github.com/ajouatom/openpilot/tree/joongyu01/carrot-wip-dev).
**내 차량 코드:** `Babysurfer-cyber/openpilot` `ms-xy-radar3`에서 파생된 PR #47 Draft.
**실제 차량 송신 실험:** 이번 변경에서는 **절대 수행하지 않음**.

## 실제 순정 EV6에서 확인한 것 (이전 rlog 6/7 분석, PR #43)

- 순정 자동주차, 실제 bus 0 SPAS2 `0x16A` (32B, CRC16-valid).
- 좌: `BLINKER_CONTROL 0→3`와 동시에 byte7 `0x82→0x01`,
  byte18 `0x00→0x40`. 0x413 실제 좌 램프 OFF→ON은 약 29.4ms 뒤.
- 우: byte7/18 전환은 `0→4`보다 약 21.6ms 선행,
  이후 0x413 실제 우 램프 OFF→ON은 약 10.2ms 뒤.
- 따라서 방향 신호 `BLINKER_CONTROL=3/4`뿐 아니라
  **차량 고유 상태 필드의 동시 변경이 관측**된다. byte7/18 값의
  인과적 의미는 확정되지 않았다.
- 기존 캡처는 외부 진단 `0x7B1` 요청이 없는 순정 시험이다.
  **ECU가 외부 요청에 응답하는지 여부는 미검증**이다.

## 원작 코드에서 바로 활용할 부분

| 원작 파일 | 내용 | PR #47 처리 |
| --- | --- | --- |
| `spas.py` | 0x165/0x16A 길이, CRC16, stock counter, 0x7B9 ACK 확인 | 수신 검증·가상 상태기계에 반영 |
| `carcontroller.py` | 좌/우 요청·중단·복구 순서 | 주행 송신 없이 모의 상태 관리에만 반영 |
| `hyundaicanfd.py:create_spas_messages` | SPAS2 원본 템플릿을 보존하면서 BYTE18을 수정하는 전략 | **수신 전용** byte7/18 진단으로 활용. CAN 메시지 생성/전송은 미이식 |
| `safety_hyundai_spas.h` | 확인된 상태에서만 TX 허용하는 구상 | panda 안전 진입 조건 검토에만 참고. 실차 allowlist는 미변경 |

## 원작의 실제 포팅 충돌 지점

1. `spas.py:frame()`은 대부분의 0x165/0x16A payload 바이트를
   **0으로 구성**하고 0x16A byte16에 3/4를 싣는다.
   EV6의 실제 factory-on 상태인 byte7=0x01, byte18=0x40과 다르다.
   이것이 작동 실패의 직접 원인이라고 아직 단정할 수는 없지만,
   단순 복사는 EV6 순정 프레임과 같지 않다.
2. `hyundaicanfd.py:create_spas_messages()`는
   `spas2_template`이 제공되면 원본 byte7 등 상태를 남기고,
   방향값 3/4일 때 byte18=0x40을 설정한다.
   그러나 **idle 시점의 원본 byte7=0x82**를 그대로 복사할 경우
   factory 방향 출력 중 관측한 **0x01**과 다르다.
   관측된 바이트 간 시간 순서도 좌/우가 다르다.
   이를 무리하게 고정 값으로 만드는 것은 아직 근거가 없다.
3. 원작의 선택적 `safety_hyundai_spas.h:hyundai_spas_tx()` 경로는
   SPAS2 CRC/counter/byte16 이외의 payload 바이트를 0으로 제한한다.
   따라서 EV6 factory-on에 관측된 **byte7=0x01, byte18=0x40**
   를 복제하면 해당 **엄격한 panda 경로에서는 거부**된다.
   이식하려면 host 메시지 생성부와 panda 안전 정책을
   함께 설계·검증해야 하며, 무작정 panda allowlist를 여는 것은 해법이 아니다.
4. 원작 `carcontroller.py` 주석에는 외부 0x7B1이 응답하지 않아
   원본 ECU와 동시에 같은 ID를 송신했을 때 CAN error/bus-off가 발생한
   사례가 있다. 다른 원작 차량 시험에서는 ACC/FCA/AVH 기능이
   SPAS2 시험 중 꺼진 사례도 있다. **이 현상을 EV6에서 직접 확인한
   것으로 혼동하면 안 된다.**
5. 원작에는 ECU ACK 없이 piggyback/direct 송신하는 mode와,
   응답을 기다리는 mode가 모두 존재한다. **ACK 없는 mode는
   이 EV6 테스트 브랜치에 이식하지 않는다.**

## 이번에 실제 적용한 코드 및 시험

`ev6_spas_control.py`의 `passive_spas2_snapshot(now)`는
CRC가 확인된 **bus 0 OEM 수신 SPAS2**에서
`blinker_control`, raw `byte7`, raw `byte18`, 수신 시간만
사본 딕셔너리로 반환한다. 원본 SPAS1/2 두 스트림의 최근
연속성도 확인한다. CRC 오류, 다른 버스, 오래된 데이터면
해당 snapshot은 불가능하다.

이 함수는 순정 데이터를 **읽기만 하며** 차량 CAN/UDS 송신 코드,
`make_can_msg`, `CanData`, panda TX 예외를 포함하지 않는다.
`source=PASSIVE_OEM_RX_ONLY`는 호출자 supplied bus metadata의
표식일 뿐 ECU 출처의 독립 인증이 아니므로
`authenticates_ecu_origin=False`로 반환한다.

- 기존 소프트웨어 상태기계 34개 + 새 read-only 진단 테스트 5개
  = **최신 39/39 Python unittest PASS** (로컬 실행).
- `py_compile` 통과. 배포 장비의 실제 ECU / panda C libsafety 테스트가 아님.
- 현재 원작의 이식 가능성만 평가한다. **실차 방향지시등 점등 실험은 하지 않음.**
- 준비된 외부 표본은 PR #43의 순정 자동주차 리포트이며,
  원본 로그의 byte7/18 의미와 ECU 인계 동작은 여전히 별개 검증 사항.

## 다음 검증 단계

1. 기존 순정 자동차 로그 또는 새로운 **수신 전용** 캡처로
   두 raw 상태 필드와 0x413 램프 피드백을 계속 비교한다.
2. 독립 ECU/CAN-FD 시험 장비에서만 0x7B9 정식 응답, OEM
   양쪽 스트림 정지 및 복귀를 실증하고 통신 오류가 없는지 확인한다.
3. 위 증거 없이는 0x7B1 진단 송신, 0x165/0x16A 중복 송신,
   주차 ECU 비활성화/재활성화, 또는 주행 중 방향지시등 자동제어를
   운행 차량에 실험하지 않는다.

**NO-GO: PR #47 Draft, `ms-xy-radar3`에 병합하거나 차량에 설치하지 않음.**
