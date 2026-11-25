# EV6 independent panda P-only 0x3C1 safety gate

## Why this PR exists

The existing 2022 EV6 blinker-stalk experiment is an **optional one-shot
0x3C1** packet. It already checks P in Python, but the original panda
safety code allowed this packet while stationary with the brake held
**without independently verifying the vehicle was in Park**.

This patch only **tightens** that existing transmit path. It does not
grant authority to send SPAS1/2 (0x165/0x16A) or APRK diagnostics (0x7B1),
and does not make external blinker control operational or road-safe.

## C safety guard

The branch uses the genuine 2022 Kia EV6 stock bus-0 ACCELERATOR/gear
message 0x35 (32 bytes, byte24 bits[2:0]) to require P=0. In the
user-supplied captures D=5, N=6, R=7.

* Fresh CRC-valid stock message; age at most 120 ms
* The panda safety framework must have **three** fresh input messages,
  0x35 (gear/gas), 0x175 (foot brake), 0xA0 (wheel speeds), each on bus 0.
  All three must have a valid checksum, acceptable counter history,
  valid quality, a non-lagging state and an age of at most 120 ms.
  Missing or unsupported messages always fail closed.
* A cached P value is insufficient: bad CRC arriving after a valid P
  causes the framework to mark the RX checksum invalid and block TX
* Zero wheel speed, foot brake held, gas not pressed, original stalk
  idle, authentic checksum and counter, 5s cooldown still required
* Failure to identify a validated bus-0 0x35 check is **fail closed**
* The existing steering and longitudinal policy is unchanged

The guard's helper uses explicit C extern declarations because policy
headers are included before the safety.h global variable definitions.

## Compiled panda tests (in this PR)

File: test_ev6_stalk_park_guard.py

22 tests written for the actual libsafety C hooks:

1. Park + brake + authentic lever tap shape (positive path)
2. D / N / R denied even with standstill and brake
3. Missing gear denied
4. Gear detected only on wrong CAN bus denied
5. Invalid gear CRC denied
6. P -> D transition revoked
7. Invalid CRC immediately after P revoked (uses RX status)
8. Safety reinitialization clears cached P
9. Stale P (>120ms) denied
10. Brake release denied
11. No authentic stock lever sample denied
12. 0x165 / 0x16A / 0x7B1 remain disallowed
13. Missing wheel-speed message denied
14. Bad foot-brake CRC after a valid brake message denied
15. Bad wheel-speed CRC after a valid message denied
16. Stale foot-brake sample rejected with other inputs fresh
17. Stale wheel-speed sample rejected with other inputs fresh
18. Wheel movement rejected despite other stationary conditions
19. No EV6 test opt-in flag means no 0x3C1 permission

The tests deliberately do not communicate with any car. A test file
present in GitHub is **not** evidence that it passed. Compile the C hooks
and run the test suite before considering this PR for any deployment.

## Commands on a provisioned developer build host

From repository root:

    cd opendbc_repo/opendbc/safety/tests
    source ../../../setup.sh
    scons -D -j"$(nproc)" --coverage
    python -m pytest -q test_ev6_stalk_park_guard.py

Then run the existing project-wide safety verification:

    ./test.sh

This environment has not successfully compiled the panda C library or
run the compiled libsafety test suite. Keep this PR **Draft** until those
results, code review and applicable panda hardware validation exist.

## Critical EV6 side effect evidence

commaai/openpilot issue #27885 (2023-04-12, 2022 EV6 GT-Line AWD HDA2)
reports loss of parking sensors, surround/blinker cameras, self-parking
and other parking-assist functions in an experimental configuration
involving parking ECU address 0x7B1. A collaborator confirmed that the
experimental turn-signal approach disabled the parking ADAS ECU and
could have those side effects:
https://github.com/commaai/openpilot/issues/27885

Do **not** combine this P-gear patch with experimental 0x7B1 disable or
duplicate-id 0x165/0x16A injection on a road vehicle. Any such experiment
requires independent isolated ECU/CAN hardware evaluation and review.

## 2026-11-25: Direct GCC source-contract verification

A separate, portable test now extracts the **actual** source of
`hyundai_ev6_parked_inputs_valid()` from the production panda header,
compiles it with GCC `-std=gnu11 -Wall -Wextra -Werror -pedantic` against
minimal test structs, and exercises 30 checks:

- valid P/gear + brake + wheel telemetry safety statuses
- missing or wrong-bus input for each of the three required stock IDs
- CRC, quality flag, counter, lagging, age and address failures
- exact 120ms freshness boundary and counter threshold

**Local standalone result: 30/30 C checks passed.** This tests the
extracted helper logic only. It is **not** a full panda firmware build
or a successful run of the 22 compiled-libsafety integration tests.

Run on a developer PC with gcc:

```bash
python opendbc_repo/opendbc/safety/tests/test_ev6_park_gate_c_contract.py
```

A dedicated CI workflow (`.github/workflows/ev6-panda-safety.yml`)
was added to compile the full C library and run both this source-contract
test and the 22 integration tests. At the time of writing GitHub
has **zero workflow runs for this PR branch**; an absent run must not
be reported as a passing build.

## Negative mutation checks

The standalone C gate contract was further checked against **four
deliberately damaged versions** of the header function. All four were
correctly rejected by the local test runner:

1. Extend the 120ms bound to 130ms (stale data accepted)
2. Bypass the RX checksum status (corrupt input accepted)
3. Change the counter threshold from strict `< MAX_WRONG_COUNTERS`
   to inclusive `<=`
4. Remove the required brake signal ID 0x175

This only establishes that the standalone checks are sensitive to these
specific regressions. The current branch still has **no confirmed remote
GitHub Actions job execution** and no full panda safety compilation.

## HDA2 버스 설정의 실제 적용 차이 (2026-11-25)

**중요한 추가 발견:** 처음 작성한 긍정 테스트는
`CANFD_LKA_STEERING | CAMERA_SCC | EV_GAS | EV6_BLINKER_STALK_TEST`
safetyParam을 사용한다. 이때 panda의 HDA2 RX 검사에는
`HYUNDAI_CANFD_COMMON_RX_CHECKS(0)`가 적용되므로 새
`hyundai_ev6_parked_inputs_valid()`가 추적하는 0x35/0x175/0xA0
**bus 0** 유효 상태와 일치한다.

반면 **일반 HDA2 / CAMERA_SCC 미설정** 구성에서는 같은
RX 검사에 `HYUNDAI_CANFD_COMMON_RX_CHECKS(1)`가 적용된다.
현재 PR #46의 독립 안전 게이트는 **bus 0**에서 세 가지 원본
메시지가 safety RX checker에 의해 확인돼야만 통과하므로,
이 구성에서는 정상 기어·브레이크·정차 정보를 수신해도
0x3C1 시험 송신은 **차단되는 것이 맞다(fail closed)**.

- 이번에 `test_ev6_stalk_park_guard.py`에 2개 테스트를 추가해
  HDA2 non-camera-SCC에서 bus0 및 bus1 수신 양쪽 모두
  0x3C1이 차단돼야 함을 명시했다.
- 총 **22개** compiled-libsafety 테스트가 작성됐으나
  **전체 C 빌드 및 21개 테스트의 실제 실행은 아직 미확인**이다.
  기존 "19개" 테스트라는 표기는 더 이상 최신 상태가 아니다.
- 실제 사용자 차량이 어떤 safetyParam 및 panda physical bus
  구성을 사용하는지는 현시점에 독립적으로 확정되지 않았다.
- 버스가 다르다는 이유로 guard를 완화하거나 RX checker를
  강제로 변경해서는 안 된다. 필요하다면 정확한 자동차 배선,
  원본 수신 경로 및 ECU 기능 안전성을 별도 검증한 후
  새 안전 설계를 검토해야 한다.

이 PR은 SPAS2 실제 점등용 코드가 아니며, 기존의 실패한
`0x3C1` 일회성 실험을 보호하는 목적이다. **현재 차량 병합 NO-GO.**


## 2026-11-25: 기본 차량 설정의 과도한 시험 플래그 수정

차량용 `interface.py`를 검토하던 중 **기존 ms-xy-radar3는
`Ev6BlinkerTest` 요청과 무관하게 모든 Kia EV6/CAN-FD의
`safetyParam`에 `EV6_BLINKER_STALK_TEST`를 설정**한다는
사실이 확인됐다. 이는 기존 문서의 "opt-in"이라는 표현과
불일치했다. 사용자 요청이 없어도 panda에서 시험용 CAN ID
0x3C1 TX 검사가 켜진 채 시작할 수 있었다.

이번 PR에서 `interface.py`를 한정적으로 변경했다.

- `candidate == CAR.KIA_EV6`, CAN-FD, `not docs`와 함께
  **`params.get_int("Ev6BlinkerTest") in (1, 2)`**인
  초기화 시점에만 panda 시험용 safetyParam을 설정한다.
- 기존 `carcontroller.py`는 선택값을 수신하면 P단 검사
  **전에 즉시 0으로 소비**하고, 조건이 충족되지 않으면
  나중에 재생하지 않는다.
- 시험용 설정값이 0이거나 정의되지 않은 기본 상태에서는
  **해당 panda TX 허용 플래그가 아예 설정되지 않는다**.
- `test_ev6_interface_optin.py`의 2개 읽기 전용 AST 검사가
  플래그의 명시적 조건과 일회성 요청 소비 순서를 확인한다.
  이 검사는 실제 C/panda 통합 검사와 구별된다.

**제한:** panda safetyParam은 기기 초기화 시 결정된다.
운행 중 `Ev6BlinkerTest`를 바꾸어도 safetyParam을 재설정하지
않으므로 송신 시험은 차단될 수 있다. 반대로 초기화 시 시험
플래그가 설정됐다면 요청 소비 후에도 그 부팅 세션에서는
panda의 다른 엄격한 안전 제한에 의해 관리되는 시험 권한이
남는다. **실차 시험 허용 또는 검증 완료라고 해석하면 안 된다.**

## 2026-11-25: 실행 경로 안전성 재점검

반복 검토에서 두 개의 실제 취약 지점을 추가로 수정했다.

1. **CI 감시 누락:** panda workflow의 push/PR 경로 필터에
   시험용 `interface.py`, `carcontroller.py` 변경이 없었다.
   두 파일과 `values.py`를 감시 대상으로 포함했다.
   이 변경은 워크플로가 실제로 실행됐다는 뜻이 아니다.
2. **오픈파일럿 제어 중 시험 명령 차단:** 기존 코드에는 P단·브레이크·
   정차 등의 제한이 있지만, `CC.enabled` / `CC.latActive` /
   `CC.longActive`가 활성화된 경우를 명시적으로 배제하지 않았다.
   호스트가 요청을 먼저 소비한 뒤 세 상태가 모두 false인지 검사하도록
   했고, **panda의 독립 TX 허용식에도 `!controls_allowed`를 추가**했다.
   Python 호스트 상태와 panda C 상태는 별개의 이중 방어이다.

새 `test_active_panda_controls_block_even_when_park_brake_and_stalk_valid`
테스트는 P/브레이크/레버 조건을 만족시킨 뒤에도 panda control
권한이 true일 때 송신을 거절해야 함을 검증하도록 작성했다.
AST 정적 계약 테스트도 호스트 세 제어 상태와 C 조건을 검사하도록
수정했다.

현재 **compiled-libsafety 테스트는 22개 작성, AST 소스 계약은
3개 작성**된 상태이며, 실제 전체 panda 컴파일·통합 실행은
아직 검증되지 않았다. 이 변경은 제한을 **강화하는 것일 뿐**
점등 성공이나 차량에서의 시험 허용을 뜻하지 않는다.
기존 실차 사용 브랜치는 수정하지 않았고, 검증 전 병합은 금지한다.

PR은 Draft이며, `ms-xy-radar3`에 병합되지 않았다.
전체 panda 빌드·21개 compiled-libsafety 테스트의 실제
통과 기록도 아직 없다. 그 전에는 차량에 설치하지 않는다.

## CI execution blocker / workflow dispatch limitation

The dedicated `.github/workflows/ev6-panda-safety.yml` currently exists
**only in this Draft PR branch**, not in default `ms-xy-radar3`.
GitHub documents that `workflow_dispatch` (the **Run workflow**
manual trigger) only receives events when its workflow definition is
present on the default branch. Therefore the button cannot be relied
on to run this unmerged new workflow.

Reference:
https://docs.github.com/en/actions/how-tos/write-workflows/choose-when-workflows-run/trigger-a-workflow

The branch `push` and `pull_request` triggers are configured but
GitHub has not reported an EV6 panda test run; this document does **not**
claim that default-branch placement is the only explanation for absent
automatic runs.

**Do not add the experimental C safety policy to the driving branch
merely to expose a CI button.** Compile and run the exact PR branch
under an isolated developer CI/workstation first, then obtain an
independent safety review. Until then, compiled C tests: NOT RUN,
vehicle operation: NO-GO.


## 2026-11-25: fresh exact-source GCC contract rerun

Following the PR #44/#45 Python test repair work, the **current PR #46**
`hyundai_ev6_parked_inputs_valid(uint32_t now)` C function was
extracted from the live GitHub Draft branch and compiled locally with
the **exact current** `test_ev6_park_gate_c_contract.py` test script.

- Source C gate: 1,068 characters, FNV32 `0xd2903584`
- Test script: 4,877 trimmed characters, FNV32 `0x2c4736c2`
- Local GCC executable run: **30/30 C source-contract assertions PASSED**
- No automotive CAN, ECU diagnostics, or TX occurred during this run.

These are transfer-consistency hashes, **not** proof of authentic ECU
provenance. The compiled helper uses minimal local ABI definitions;
**the full panda `libsafety.so` build, 22 compiled-libsafety tests, and
hardware panda tests remain UNRUN/UNVERIFIED**. No remote GitHub
Actions success has been observed. The original experimental 0x3C1
transmit path still must NOT be deployed to or tested on the road car.
