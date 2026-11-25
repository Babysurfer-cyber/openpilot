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
3. Wait for simulated positive communication-disable response (0x68 0x03).
4. Wait for *both* OEM SPAS message streams to be quiet for at least 100 ms.
5. Grant *bench-only* eligibility for one second, revoked immediately on
   stock SPAS resumption, bus fault, driver action, gear/brake change.
6. Simulate separate enable (0x68 0x00) and default-session (0x50 0x01)
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
