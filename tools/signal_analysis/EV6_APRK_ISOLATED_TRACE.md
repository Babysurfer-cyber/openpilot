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
- Independent CAN-FD analyzer health:
  {"ms":20,"direction":"health","bus":0,"bus_off":false,"errors":0,"tx_lost":0,"rx_lost":0}

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
- Independent before/after CAN-FD health samples with no bus-off or error
  counter increase. Reject any externally generated SPAS in the passive probe.
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
