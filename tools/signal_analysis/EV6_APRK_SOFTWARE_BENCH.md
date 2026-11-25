# EV6 APRK / SPAS handoff: software-only bench

This directory contains a pure Python *hypothetical* protocol model, not a
controller, safety firmware change, diagnostic session driver, or vehicle test.
It never sends CAN or UDS frames and is not imported by onroad services.

## Evidence from factory EV6 recordings

- Auto-park left (rlog 6) and right (rlog 7) illuminate the turn lamps
  with SPAS2 address 0x16A and BLINKER_CONTROL values 3 and 4.
- Regular parked (rlog 9) shows SPAS1 0x165 and SPAS2 0x16A
  continuing near 50 Hz with BLINKER_CONTROL 0.
- Previous PR #43 offline report counted 9,238 SPAS1 and 9,239 SPAS2
  factory messages with zero CRC errors across these three captures.
- The captures contain no 0x7B9 diagnostic response, **but they also contain
  no external diagnostic-session request**. Thus ECU handoff support is
  UNKNOWN, neither proven nor disproven.

## Simulated fail-closed handoff

1. Require P, standstill, foot brake, no gas/driver lever/lamp, healthy CAN
   and ten recent consecutive factory SPAS1/SPAS2 messages.
2. Wait for the exact simulated positive session response (0x50 0x03).
3. In a distinct `wait_post_session_stock` phase, require **both**
   CRC-valid, advancing factory SPAS1 and SPAS2 streams **after session ACK**
   within a bounded 50 ms observation window. Until both have been seen,
   the mock must **NOT emit** `WOULD_REQUEST_COMMUNICATION_DISABLE`.
   Only then does it record the hypothetical disable request timestamp.
   An already silent factory ECU must not be mistaken for a handoff.
4. Wait for simulated positive communication-disable response (0x68 0x03).
   This ACK may arrive later than the 50 ms **pre-request** freshness
   window: the evidence was required at request time, not ACK time.
   Reject invalid OEM frames during transition; if stock SPAS continues at
   or after the **100 ms disable-grace boundary**, immediately revoke the
   mock handoff rather than waiting indefinitely for eventual silence.
5. Wait for *both* OEM SPAS streams to be quiet for at least 100 ms.
6. Grant *bench-only* eligibility for one second, revoked immediately on
   stock SPAS resumption, bus fault, driver action, gear/brake change.
7. Simulate separate enable (0x68 0x00) and default-session (0x50 0x01)
   replies and the stock stream returning; otherwise restoration is
   unconfirmed and never marked safe.

The model produces only string labels for WOULD_REQUEST actions. There is
no vehicle transport and the real CAN transmit count is always zero.

## Run

From repository root using Python 3.10+:

    python -m unittest discover -s tools/signal_analysis -p test_ev6_aprk_handoff_bench.py -v

    python tools/signal_analysis/ev6_aprk_handoff_bench.py --report /path/to/ev6_spas_oem_report.json --output ev6_aprk_handoff_report.json

Omitting --report uses only synthetic mock ECU responses.

## Before any real vehicle actuation

An isolated ECU/CAN test rig, independent bus fault/error counters, physical
power cutoff, and a safety review are required. Validate both positive and
negative diagnostics, OEM source silence, no duplicated-ID contention, and
full restoration before considering a separate parked vehicle experiment.
Donor implementations recorded SPAS CAN errors and bus-offs; a piggyback
scheduler by itself is not proof of safe bus arbitration.

## 2026-11-25 verification alignment with PR #45

The earlier simulated state machine could grant a lease after late OEM
SPAS traffic finally stopped, even though PR #45 rejects a stock message
seen at least 100 ms after the disable acknowledgement. Its original
positive fixture also proceeded with *no* OEM SPAS traffic after session
ACK, so an ECU already silent during session entry could be mistaken for
a successful communication disable.

The simulation now fails closed on these contradictions and on invalid
stock frames during the session-to-disable and quiet periods. The positive
fixture and CLI example were updated to include genuine-looking **synthetic**
post-session stream liveness. The PR contains **25 test methods** after
adding focused negative cases. The latest GitHub revision has **not been
executed as a complete test suite** in the available environment, and
has no passing GitHub Actions run. The code is an unverified **offline mock**,
not a vehicle ECU implementation or permission to merge for road use.

## 2026-11-25: pre-request ordering correction

A subsequent code review found the older mock emitted
`WOULD_REQUEST_COMMUNICATION_DISABLE` **immediately** on accepting the
extended-session ACK. It then demanded factory stream activity at the
later *disable ACK*, which is **not** the equivalent of PR #45's
"both OEM streams alive before the disable request" invariant.

The mock now waits for genuine-shaped, sequential factory stream
observations from BOTH `0x165` and `0x16A` following session ACK,
with a fail-closed 50 ms observation window; **only after** both have
been confirmed does it record a hypothetical disable request.
An early fabricated disable ACK cannot advance the state machine.
A delayed positive disable ACK may still be accepted based on valid
**pre-request** evidence.

The current branch has **25 written pure-Python mock test methods**.
Latest GitHub HEAD syntax/behavior has not been executed end-to-end in
the local runtime, because a complete live repo checkout is unavailable
there; no GitHub Actions run has been confirmed. Source inspection
and consistency checks are **not** a passing test run. Everything
here remains an OFFLINE simulation that sends **zero** real CAN frames.

## 2026-11-25: execution-grade validation after runtime NameError fix

A direct Python execution check (not AST inspection) exposed a genuine
runtime `NameError` at the post-session OEM stream-continuity gate:
the local boolean was named `consecutive`, but the condition referred
to the undefined name `continuous`. The mock could crash instead of
entering its safe failure path. This was fixed in production source.

Restore-phase negative ECU diagnostic responses were also found to
be ignored by the existing generic `_abort()` return guard.
A negative `0x7B9` during communication-enable or default-session
restore now immediately enters **`restore_unconfirmed`**, and subsequent
positive replies cannot override that terminal state.

**Direct local execution, after fixes: 27/27 current PR #44 unittest
cases PASSED**, Python 3.13.5. The temporary local test copy and the
production class/report functions were independently checked against
the exact latest GitHub branch source, with matching character counts
and FNV32 content hashes:

- Python source section (`@dataclass` through `summarize_oem_report`):
  10,860 characters, hash `0xb8048d34`
- `test_ev6_aprk_handoff_bench.py`: 11,174 characters,
  hash `0xcfdb4513`

These hashes are simple verification checks for this transfer, **not**
cryptographic authenticity or a hardware trace audit. This runtime
result supersedes earlier notes saying current tests were unexecuted;
those notes describe intermediate revisions. There is **still no
GitHub Actions CI run** for this branch, and no panda full C build or
isolated physical APRK ECU test. **Vehicle use / merge remains NO-GO.**
