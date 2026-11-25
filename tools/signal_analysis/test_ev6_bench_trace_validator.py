"""Synthetic isolated-bench captures: never creates real CAN sockets."""
import unittest
from ev6_bench_trace_validator import REQ, SPAS, analyze, crc16_xmodem


def e(t, kind, addr=0, data="", bus=0, **kw):
    return {"ms": t, "direction": kind, "address": hex(addr),
            "data": data, "bus": bus, **kw}


def stock_frame(addr, counter):
    size, xor = SPAS[addr]
    data = bytearray(size)
    data[2] = counter
    data[:2] = (crc16_xmodem(data[2:] + addr.to_bytes(2, "little")) ^ xor).to_bytes(2, "little")
    return bytes(data).hex()


def health(t, errors=0, **kwargs):
    return e(t, "health", errors=errors, tx_lost=0, rx_lost=0, **kwargs)


def traffic():
    rows = [health(0)]
    for i in range(10):
        for addr in SPAS:
            rows.append(e(20*i, "rx", addr, stock_frame(addr, i)))
    rows += [e(190, "tx", 0x7b1, REQ["session"]),
             e(210, "rx", 0x7b9, "0250030000000000"),
             e(220, "tx", 0x7b1, REQ["disable"]),
             e(240, "rx", 0x7b9, "0268030000000000"),
             e(350, "tx", 0x7b1, REQ["enable"]),
             e(370, "rx", 0x7b9, "0268000000000000"),
             e(380, "tx", 0x7b1, REQ["default"]),
             e(400, "rx", 0x7b9, "0250010000000000")]
    for i in range(10):
        for addr in SPAS:
            rows.append(e(420+i*20, "rx", addr, stock_frame(addr, 10+i)))
    rows.append(health(610))
    return rows


class TestEvidence(unittest.TestCase):
    def test_clean_simulated_trace(self):
        r = analyze(traffic())
        self.assertEqual(r["diagnostic"], "ISOLATED_BENCH_HANDOFF_AND_RESTORE_OBSERVED")
        self.assertFalse(r["permits_vehicle_actuation"])
        self.assertEqual(r["vehicle_can_tx"], 0)

    def test_no_session_request_is_not_tested(self):
        rows = [r for r in traffic() if r["address"] not in ("0x7b1", "0x7b9")]
        self.assertEqual(analyze(rows)["diagnostic"], "NOT_TESTED_NO_EXTERNAL_SESSION_REQUEST")

    def test_wrong_or_unsolicited_response(self):
        rows = traffic()
        rows[22]["data"] = "0250010000000000"
        self.assertIn("UNSOLICITED_WRONG_OR_LATE_ECU_RESPONSE", analyze(rows)["faults"])

    def test_session_ack_on_echo_bus_not_valid(self):
        rows = traffic()
        reply = next(r for r in rows if r["address"] == "0x7b9")
        reply["bus"] = 128
        self.assertIn("REQUEST_BEFORE_REQUIRED_POSITIVE_ACK:disable", analyze(rows)["faults"])

    def test_disable_ack_missing(self):
        rows = traffic()
        rows = [r for r in rows if not (r["ms"] == 240 and r["address"] == "0x7b9")]
        self.assertIn("ECU_COMMUNICATION_DISABLE_NOT_ACKNOWLEDGED", analyze(rows)["faults"])

    def test_stock_continues_after_grace(self):
        rows = traffic()
        rows.insert(25, e(345, "rx", 0x16a, stock_frame(0x16a, 10)))
        self.assertIn("OEM_SPAS_STILL_TRANSMITTING_AFTER_DISABLE_GRACE", analyze(rows)["faults"])

    def test_restore_no_stock(self):
        rows = [r for r in traffic() if not
                (r["address"] in ("0x165", "0x16a") and r["ms"] > 400)]
        self.assertIn("STOCK_SPAS_NOT_RESTORED_WITH_VALID_COUNTERS", analyze(rows)["faults"])

    def test_external_spas_injection_rejected(self):
        rows = traffic()
        rows.insert(25, e(345, "tx", 0x16a, stock_frame(0x16a, 10)))
        self.assertIn("EXTERNAL_SPAS_TRANSMISSION_PRESENT_IN_PASSIVE_PROBE", analyze(rows)["faults"])

    def test_bus_error_increment(self):
        rows = traffic()
        rows[-1]["errors"] = 1
        self.assertIn("CAN_ERROR_COUNTER_INCREASED:errors", analyze(rows)["faults"])

    def test_missing_health(self):
        rows = [r for r in traffic() if r["direction"] != "health"]
        self.assertIn("NO_INDEPENDENT_BEFORE_AFTER_CAN_HEALTH_MONITOR", analyze(rows)["faults"])

    def test_negative_ack(self):
        rows = traffic()
        rows[22]["data"] = "037f102200000000"
        self.assertIn("ECU_NEGATIVE_RESPONSE", analyze(rows)["faults"])

    def test_spas_bad_crc(self):
        rows = traffic()
        rows[4]["data"] = "0000" + rows[4]["data"][4:]
        self.assertIn("INVALID_STOCK_SPAS_CRC_OR_LENGTH", analyze(rows)["faults"])

    def test_timestamps_monotonic(self):
        rows = traffic()
        rows[5]["ms"] = -1
        self.assertIn("NONMONOTONIC_CAPTURE_TIMESTAMPS", analyze(rows)["faults"])

    def test_reply_not_sufficient_without_restore(self):
        rows = [r for r in traffic() if r["ms"] < 350]
        self.assertIn("ECU_RESTORE_NOT_ATTEMPTED", analyze(rows)["faults"])


if __name__ == "__main__":
    unittest.main()
