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

19 tests written for the actual libsafety C hooks:

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
