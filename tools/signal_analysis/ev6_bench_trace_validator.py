#!/usr/bin/env python3
"""Read-only EV6 APRK/SPAS handoff evidence validator for ISOLATED bench traces.

Consumes JSONL observations exported by an independent CAN-FD bench monitor.
No imports for CAN interfaces, sockets, panda, or UDS send functions.
This cannot grant permission for vehicle actuation.

Usage:
    python ev6_bench_trace_validator.py bench_trace.jsonl --out verdict.json

JSONL schema:
  {"ms":0.0,"direction":"rx","bus":0,"address":"0x165","data":"HEX"}
  {"ms":0.0,"direction":"tx","bus":0,"address":"0x7b1","data":"0210030000000000"}
  {"ms":0.0,"direction":"health","bus":0,"bus_off":false,"errors":0,"tx_lost":0,"rx_lost":0}

Only observations on bus 0 count for a protocol proof. rx denotes a real
bus reception, not sendcan or host transmit echo. Bus 128/192 cannot
be mistaken for actual ECU responses. Factory SPAS CRC/counter required.
"""
import argparse
import json
import math
from pathlib import Path

SPAS = {0x165: (24, 0x819D), 0x16A: (32, 0x9F5B)}
REQUEST, RESPONSE = 0x7B1, 0x7B9
REQ = {"session": "0210030000000000", "disable": "0328030100000000",
       "enable": "0328000100000000", "default": "0210010000000000"}
ACK = {"session": "5003", "disable": "6803", "enable": "6800", "default": "5001"}
STEPS = ("session", "disable", "enable", "default")


def crc16_xmodem(buf):
    v = 0
    for b in buf:
        v ^= b << 8
        for _ in range(8):
            v = ((v << 1) ^ (0x1021 if v & 0x8000 else 0)) & 0xffff
    return v


def valid_stock(addr, payload):
    if addr not in SPAS or len(payload) != SPAS[addr][0]:
        return False
    return int.from_bytes(payload[:2], "little") == (crc16_xmodem(payload[2:] + addr.to_bytes(2, "little")) ^ SPAS[addr][1])


def parse_address(value):
    return value if isinstance(value, int) else int(value, 0)


def decode_reply(raw):
    if len(raw) != 8 or not 2 <= raw[0] <= 7:
        return None
    return raw[1:1 + raw[0]].hex()


def analyze(events):
    out = {"kind": "EV6_ISOLATED_BENCH_PASSIVE_VERIFIER", "vehicle_can_tx": 0,
           "permits_vehicle_actuation": False, "diagnostic": "NO_REQUEST_OBSERVED", "proof": {},
           "faults": [], "stage": "stock", "external_spas_tx": 0,
           "offline_verifier_can_tx": 0, "captured_aprk_requests": 0,
           "evidence_is_authenticated": False}
    faults, proof = out["faults"], out["proof"]
    stage = "stock"
    sent = {}
    acked = {}
    last_stock = {a: None for a in SPAS}
    prev_counter = {a: None for a in SPAS}
    valid_count = {a: 0 for a in SPAS}
    first_health = None
    health_samples = 0
    health_records = []
    last_ms = -1.0
    disable_ack_ms = None
    stock_after_session_ack = {a: None for a in SPAS}
    default_ack_ms = None
    last_good_stock_after_default = {a: None for a in SPAS}
    stock_resume_count = {a: 0 for a in SPAS}
    last_resume_counter = {a: None for a in SPAS}

    def fault(label):
        if label not in faults:
            faults.append(label)

    for event in events:
        if not isinstance(event, dict):
            fault("INVALID_CAPTURE_EVENT")
            continue
        try:
            ms = float(event["ms"])
        except (ValueError, TypeError, KeyError):
            fault("INVALID_CAPTURE_TIMESTAMP")
            continue
        if not math.isfinite(ms) or ms < 0:
            fault("INVALID_CAPTURE_TIMESTAMP")
            continue
        if ms < last_ms:
            fault("NONMONOTONIC_CAPTURE_TIMESTAMPS")
        last_ms = ms
        kind = event.get("direction")
        if "bus" not in event or type(event["bus"]) is not int or event["bus"] < 0:
            fault("MISSING_OR_INVALID_CAN_BUS")
            continue
        bus = event["bus"]
        if kind == "health":
            if bus != 0:
                continue
            keys = ("bus_off", "error_warning", "error_passive", "errors", "tx_lost", "rx_lost", "capture_dropped")
            if any(key not in event for key in keys):
                fault("INCOMPLETE_CAN_HEALTH_SAMPLE")
                continue
            if any(type(event[key]) is not bool for key in keys[:3]) or any(
                type(event[key]) is not int or event[key] < 0 for key in keys[3:]
            ):
                fault("INVALID_CAN_HEALTH_SAMPLE")
                continue
            health_samples += 1
            h = {key: event[key] for key in keys[3:]}
            health_records.append((ms, h))
            if first_health is None:
                first_health = h
            if event["bus_off"] or event["error_warning"] or event["error_passive"]:
                fault("CAN_CONTROLLER_REPORTED_ERROR_STATE")
            for metric in h:
                if h[metric] > first_health[metric]:
                    fault("CAN_ERROR_COUNTER_INCREASED:" + metric)
                if h[metric] < first_health[metric]:
                    fault("CAN_COUNTER_RESET_OR_UNTRUSTWORTHY:" + metric)
            continue
        if kind not in ("rx", "tx"):
            fault("INVALID_EVENT_DIRECTION")
            continue
        try:
            addr = parse_address(event["address"])
            data = bytes.fromhex(event["data"])
        except (ValueError, TypeError, KeyError):
            fault("INVALID_CAN_ADDRESS_OR_DATA")
            continue
        if addr < 0 or addr > 0x1FFFFFFF:
            fault("INVALID_CAN_ADDRESS_OR_DATA")
            continue
        if bus != 0:
            # Host echo on bus128 is not evidence of an ECU response.
            continue

        if kind == "tx" and addr in SPAS:
            out["external_spas_tx"] += 1
            fault("EXTERNAL_SPAS_TRANSMISSION_PRESENT_IN_PASSIVE_PROBE")
            continue

        if kind == "rx" and addr in SPAS:
            if not valid_stock(addr, data):
                valid_count[addr] = 0
                fault("INVALID_STOCK_SPAS_CRC_OR_LENGTH")
                continue
            counter = data[2]
            continuous = (last_stock[addr] is not None and ms - last_stock[addr] < 100 and
                          counter == (prev_counter[addr] + 1) % 256)
            if last_stock[addr] is not None and 0 <= ms - last_stock[addr] < 100 and not continuous:
                fault("STOCK_COUNTER_DISCONTINUITY")
            valid_count[addr] = valid_count[addr] + 1 if continuous else 1
            last_stock[addr] = ms
            prev_counter[addr] = counter
            if "session" in acked and "disable" not in sent and ms >= acked["session"]:
                stock_after_session_ack[addr] = ms
            if stage == "quiet":
                if disable_ack_ms is not None and ms >= disable_ack_ms + 100:
                    fault("OEM_SPAS_STILL_TRANSMITTING_AFTER_DISABLE_GRACE")
            if default_ack_ms is not None and ms > default_ack_ms:
                continuous_restore = (last_good_stock_after_default[addr] is not None and
                                      0 < ms - last_good_stock_after_default[addr] < 100 and
                                      last_resume_counter[addr] is not None and
                                      counter == (last_resume_counter[addr] + 1) % 256)
                stock_resume_count[addr] = stock_resume_count[addr] + 1 if continuous_restore else 1
                last_good_stock_after_default[addr] = ms
                last_resume_counter[addr] = counter
            continue

        if kind == "tx" and addr == REQUEST:
            out["captured_aprk_requests"] += 1
            action = next((name for name, p in REQ.items() if data.hex() == p), None)
            if action is None:
                fault("UNEXPECTED_APRK_DIAGNOSTIC_REQUEST")
                continue
            expected = {"stock": "session", "session": "disable", "quiet": "enable", "enable": "default"}.get(stage)
            if action != expected:
                fault("UNEXPECTED_REQUEST_ORDER:" + action)
                continue
            previous = {"disable": "session", "default": "enable", "enable": "disable"}.get(action)
            if previous is not None and previous not in acked:
                fault("REQUEST_BEFORE_REQUIRED_POSITIVE_ACK:" + action)
                continue
            if action == "session":
                if not all(valid_count[a] >= 10 and last_stock[a] is not None and
                           0 <= ms - last_stock[a] < 100 for a in SPAS):
                    fault("INSUFFICIENT_VALID_OEM_SPAS_BEFORE_PROBE")
                out["diagnostic"] = "REQUEST_OBSERVED"
                stage = "session"
            elif action == "disable":
                # Require evidence that BOTH genuine OEM SPAS streams were
                # active during the session after the positive ACK and shortly
                # before communication-disable. Otherwise a silent/unplugged
                # ECU could be mistaken for a successful handoff.
                if any(stock_after_session_ack[a] is None or
                       not (0 <= ms - stock_after_session_ack[a] <= 50)
                       for a in SPAS):
                    fault("OEM_SPAS_NOT_ACTIVE_IMMEDIATELY_BEFORE_DISABLE")
                proof["stock_active_after_session_ack_before_disable"] = all(
                    stock_after_session_ack[a] is not None and
                    0 <= ms - stock_after_session_ack[a] <= 50 for a in SPAS)
                stage = "disable"
            elif action == "enable":
                # An indefinitely silent ECU is a separate fault, not a
                # permanent delegated lease. Only short, bounded probes count.
                if disable_ack_ms is not None and ms - disable_ack_ms > 500:
                    fault("UNBOUNDED_OEM_QUIET_INTERVAL")
                if disable_ack_ms is None or ms - disable_ack_ms < 100 or any(
                    ms - last_stock[a] < 100 for a in SPAS if last_stock[a] is not None):
                    fault("OEM_SPAS_NOT_BOTH_QUIET_BEFORE_RESTORE")
                proof["quiet_period_verified_before_restore"] = not any(
                    ms - last_stock[a] < 100 for a in SPAS if last_stock[a] is not None)
                stage = "enable"
            elif action == "default":
                stage = "default"
            sent[action] = ms
            continue

        if kind == "rx" and addr == RESPONSE:
            reply = decode_reply(data)
            if reply is None:
                fault("MALFORMED_ECU_RESPONSE")
                continue
            if reply.startswith("7f"):
                fault("ECU_NEGATIVE_RESPONSE")
                continue
            wanted = {"session": "session", "disable": "disable", "enable": "enable", "default": "default"}.get(stage)
            if wanted is None or wanted not in sent or not reply.startswith(ACK[wanted]) or \
               not (0 <= ms - sent[wanted] <= 400):
                fault("UNSOLICITED_WRONG_OR_LATE_ECU_RESPONSE")
                continue
            acked[wanted] = ms
            proof[wanted + "_response_latency_ms"] = round(ms - sent[wanted], 2)
            if wanted == "session":
                stage = "session"
            elif wanted == "disable":
                disable_ack_ms = ms
                stage = "quiet"
            elif wanted == "enable":
                stage = "enable"
            elif wanted == "default":
                default_ack_ms = ms
                stage = "restore"
            continue

    out["stage"] = stage
    proof["verified_acks"] = sorted(acked)
    proof["stock_resume_counts"] = {hex(k): v for k, v in stock_resume_count.items()}
    proof["health_samples"] = health_samples
    if health_samples < 2:
        fault("NO_INDEPENDENT_BEFORE_AFTER_CAN_HEALTH_MONITOR")
    proof["health_sample_times_ms"] = [t for t, _ in health_records]
    if "session" in sent:
        # A silent trace proves nothing if the independent monitor stopped
        # collecting. Require uninterrupted error/loss telemetry THROUGH the
        # OEM quiet interval and AFTER the last factory restoration frame.
        start_ms = sent["session"]
        recovered = [x for x in last_good_stock_after_default.values() if x is not None]
        end_ms = max([start_ms] + recovered + list(acked.values()))
        if not health_records or not any(t <= start_ms for t, _ in health_records):
            fault("NO_HEALTH_SAMPLE_BEFORE_SESSION")
        if not health_records or not any(t >= end_ms for t, _ in health_records):
            fault("NO_HEALTH_SAMPLE_AFTER_RESTORE")
        if health_records:
            relevant = [t for t, _ in health_records if t >= max((t for t, _ in health_records if t <= start_ms), default=-1)]
            for a, b in zip(relevant, relevant[1:]):
                if a < end_ms and b - a > 100:
                    fault("HEALTH_MONITOR_COVERAGE_GAP")
        if disable_ack_ms is not None and "enable" in sent:
            if not any(disable_ack_ms <= t <= sent["enable"] for t, _ in health_records):
                fault("NO_HEALTH_SAMPLE_DURING_OEM_SILENCE")
    if "session" in sent and "session" not in acked:
        fault("ECU_EXTENDED_SESSION_NOT_ACKNOWLEDGED")
    if "disable" in sent and "disable" not in acked:
        fault("ECU_COMMUNICATION_DISABLE_NOT_ACKNOWLEDGED")
    if "enable" in sent and "enable" not in acked:
        fault("ECU_COMMUNICATION_ENABLE_NOT_ACKNOWLEDGED")
    if "default" in sent and "default" not in acked:
        fault("ECU_DEFAULT_SESSION_NOT_ACKNOWLEDGED")
    if "disable" in acked and "enable" not in sent:
        fault("ECU_RESTORE_NOT_ATTEMPTED")
    if "default" in acked and any(stock_resume_count[a] < 10 for a in SPAS):
        fault("STOCK_SPAS_NOT_RESTORED_WITH_VALID_COUNTERS")
    if set(acked) == set(STEPS) and not faults and all(stock_resume_count[a] >= 10 for a in SPAS):
        out["diagnostic"] = "ISOLATED_BENCH_HANDOFF_AND_RESTORE_OBSERVED"
        out["stage"] = "complete"
    elif out["diagnostic"] != "NO_REQUEST_OBSERVED":
        out["diagnostic"] = "INCOMPLETE_OR_UNSAFE_BENCH_TRACE"
    else:
        out["diagnostic"] = "NOT_TESTED_NO_EXTERNAL_SESSION_REQUEST"
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("jsonl", type=Path)
    p.add_argument("--out", type=Path)
    args = p.parse_args()
    with args.jsonl.open(encoding="utf-8") as f:
        verdict = analyze(json.loads(line) for line in f if line.strip())
    payload = json.dumps(verdict, ensure_ascii=False, indent=2) + "\n"
    if args.out:
        args.out.write_text(payload, encoding="utf-8")
    print(payload)


if __name__ == "__main__":
    main()
