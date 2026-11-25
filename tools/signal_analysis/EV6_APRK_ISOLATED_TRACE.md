# EV6 APRK/SPAS isolated bench — passive trace evidence gates

This is a **read-only post-processing tool** for an **isolated ECU/CAN-FD bench**.
It has no CAN driver, does not call sendcan, does not run in openpilot onroad,
does not change panda safety and **cannot operate the vehicle**.

It follows the previous simulated ECU-handoff work in PR #44. A successful
**synthetic** trace is only evidence that the validator recognizes the
specified sequence; it does not prove a real EV6 can respond or that lamp
control is safe on the car.

## Prior evidence

- The user's EV6 factory auto-parking rlogs 6 and 7 captured SPAS2 0x16A
  with BLINKER_CONTROL 3 (left) and 4 (right), actual lamp feedback.
- The normal idle rlog 9 captured SPAS1/2 continuing ~50 Hz with control 0.
- The recordings contain **no external 0x7B1 diagnostic-session request**.
  Therefore absence of 0x7B9 is **NOT_TESTED**, not evidence of rejection.
- Stock SPAS 0x165/0x16A intervals are typically ~20ms with variable jitter.
  Sending another frame with the same ID into this stream is **not safe by
  assumption**. Donor implementations logged CAN errors and bus-offs.

## Offline input

Export an independent, accurately timestamped bench capture as JSONL with
one event per line, in nondecreasing monotonic millisecond order:

- Stock ECU bus traffic: {"ms":10,"direction":"rx","bus":0,"address":"0x165","data":"HEX"}
- Bench controller diagnostic request OBSERVATION:
  {"ms":15,"direction":"tx","bus":0,"address":"0x7b1","data":"HEX"}
- Independent CAN-FD analyzer health (all fields mandatory):
  {"ms":20,"direction":"health","bus":0,"bus_off":false,"error_warning":false,"error_passive":false,"errors":0,"tx_lost":0,"rx_lost":0,"capture_dropped":0}

The **bus field is mandatory and must be a nonnegative integer on every
record**, including health telemetry and ECU replies. In older versions a
missing bus value was silently treated as bus 0, which could falsely accept
a purported ECU reply. Version with this fix fails closed instead. A wrong
payload, missing address, invalid hex or unexpected SPAS stock counter jump
also fails the bench verdict.

The validator itself sends **zero** CAN packets. The
`captured_aprk_requests` output counts diagnostic transmissions observed
**in the supplied bench trace** (which may be greater than zero); it is not
the validator's own transmit count. The `evidence_is_authenticated=false`
field highlights that JSONL alone cannot establish ECU or instrument
provenance. Synthetic successes are never real ECU evidence.

Capture direction **rx** means actual bus receive, not host sendcan, return
echo bus 128, or rejected bus 192. Real bench equipment and an isolated OEM
ECU/power source are prerequisites. DO NOT run ECU-session or communication
disable requests in the connected vehicle to generate this trace.

## How to use

From repository root, using Python 3.10+:

    python -m unittest discover -s tools/signal_analysis -p 'test_ev6_bench_trace_validator.py' -v

    python tools/signal_analysis/ev6_bench_trace_validator.py isolated_bench_capture.jsonl --out isolated_verdict.json

## Evidence required

- Genuine fresh, CRC-valid, advancing stock SPAS1 and SPAS2 on the correct bus
  before requesting an ECU diagnostic session.
- Correlated ECU positive response **after** each session and communication
  mode request. Wrong bus, stale or unsolicited responses are not proof.
- Independent evidence of **both** original stock SPAS streams stopping for
  at least 100ms after acknowledgment. A synthetic positive response is
  insufficient.
- Restore both ECU communication and default session, receive positive
  responses and see 10 fresh consecutive OEM SPAS frames per address again.
- Independent CAN-FD health samples with explicit nonnegative integer counters
  (`errors`, `tx_lost`, `rx_lost`, `capture_dropped`) and true boolean state flags;
  **no default values** for absent fields. Reject any increase or reset.
- Health samples must cover the period before the session request, the ECU
  silence interval, and after the last restored OEM SPAS frame, with no
  monitoring interval exceeding **100 ms**. Missing coverage, including an
  unobserved gap during the apparent OEM silence, is not proof of silence.
- Reject any externally generated SPAS in the passive probe.
- Fail closed on missing response, invalid CRC, stale data, continuing stock,
  absent restoration, or CAN error counters.

A verdict of ISOLATED_BENCH_HANDOFF_AND_RESTORE_OBSERVED only classifies the
**provided capture**. It is not a safe-to-transmit authorization for a car.
The verifier does not and cannot independently authenticate which physical
device sent a CAN frame. The independent bench setup must establish provenance.

## Results from the existing rlogs

The factory rlogs, by design, have **no external diagnostic session request**,
so the correct conclusion is NOT_TESTED_NO_EXTERNAL_SESSION_REQUEST. The real
next experiment requires a physical isolated ECU/CAN bench and trained
supervision. The old rlogs or a more detailed software simulation cannot
replace that evidence.

## Review finding corrected in PR #45

The original validator could report a successful synthetic handoff even when
all CAN health counters were absent or both health samples were before the
session. The updated parser fails closed on missing fields, type errors,
counter reset/increase, missing post-restore sample and >100ms monitor gaps.
The added negative cases are regression-tested; the tests still use synthetic
capture events, NOT a real parking ECU. An independent monitor cannot prove
physical CAN origin from ID alone, so an isolated hardware setup is essential.
