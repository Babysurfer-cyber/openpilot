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

21 tests written for the actual libsafety C hooks:

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
or a successful run of the 21 compiled-libsafety integration tests.

Run on a developer PC with gcc:

```bash
python opendbc_repo/opendbc/safety/tests/test_ev6_park_gate_c_contract.py
```

A dedicated CI workflow (`.github/workflows/ev6-panda-safety.yml`)
was added to compile the full C library and run both this source-contract
test and the 21 integration tests. At the time of writing GitHub
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
- 총 **21개** compiled-libsafety 테스트가 작성됐으나
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
