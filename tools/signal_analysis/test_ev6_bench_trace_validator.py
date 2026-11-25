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
    record = e(t, "health", bus_off=False, error_warning=False, error_passive=False,
               errors=errors, tx_lost=0, rx_lost=0, capture_dropped=0)
    record.update(kwargs)
    return record


def traffic():
    rows = [health(t) for t in range(0, 651, 50)]
    for i in range(10):
        for addr in SPAS:
            rows.append(e(20*i, "rx", addr, stock_frame(addr, i)))
    rows += [e(190, "tx", 0x7b1, REQ["session"]),
             e(210, "rx", 0x7b9, "0250030000000000"),
             e(212, "rx", 0x165, stock_frame(0x165, 10)),
             e(214, "rx", 0x16a, stock_frame(0x16a, 10)),
             e(220, "tx", 0x7b1, REQ["disable"]),
             e(240, "rx", 0x7b9, "0268030000000000"),
             e(350, "tx", 0x7b1, REQ["enable"]),
             e(370, "rx", 0x7b9, "0268000000000000"),
             e(380, "tx", 0x7b1, REQ["default"]),
             e(400, "rx", 0x7b9, "0250010000000000")]
    for i in range(10):
        for addr in SPAS:
            rows.append(e(420+i*20, "rx", addr, stock_frame(addr, 10+i)))
    rows.sort(key=lambda row: row["ms"])
    return rows


class TestEvidence(unittest.TestCase):
    def test_clean_simulated_trace(self):
        r = analyze(traffic())
        self.assertEqual(r["diagnostic"], "ISOLATED_BENCH_HANDOFF_AND_RESTORE_OBSERVED")
        self.assertFalse(r["permits_vehicle_actuation"])
        self.assertIsNone(r["vehicle_can_tx"])

    def test_no_session_request_is_not_tested(self):
        rows = [r for r in traffic() if r["address"] not in ("0x7b1", "0x7b9")]
        self.assertEqual(analyze(rows)["diagnostic"], "NOT_TESTED_NO_EXTERNAL_SESSION_REQUEST")

    def test_wrong_or_unsolicited_response(self):
        rows = traffic()
        next(r for r in rows if r["ms"] == 210 and r["address"] == "0x7b9")["data"] = "0250010000000000"
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
        rows.append(e(345, "rx", 0x16a, stock_frame(0x16a, 10)))
        rows.sort(key=lambda r: r["ms"])
        self.assertIn("OEM_SPAS_STILL_TRANSMITTING_AFTER_DISABLE_GRACE", analyze(rows)["faults"])

    def test_restore_no_stock(self):
        rows = [r for r in traffic() if not
                (r["address"] in ("0x165", "0x16a") and r["ms"] > 400)]
        self.assertIn("STOCK_SPAS_NOT_RESTORED_WITH_VALID_COUNTERS", analyze(rows)["faults"])

    def test_external_spas_injection_rejected(self):
        rows = traffic()
        rows.append(e(345, "tx", 0x16a, stock_frame(0x16a, 10)))
        rows.sort(key=lambda r: r["ms"])
        self.assertIn("EXTERNAL_SPAS_TRANSMISSION_PRESENT_IN_PASSIVE_PROBE", analyze(rows)["faults"])

    def test_external_spas_tx_on_another_physical_bus_must_fail(self):
        # This was falsely classified as a valid bench handoff before the
        # physical-bus TX check was moved ahead of the bus-0 evidence filter.
        for bus in (1, 2, 7):
            with self.subTest(bus=bus):
                rows = traffic()
                rows.append(e(345, "tx", 0x16a, stock_frame(0x16a, 10), bus=bus))
                rows.sort(key=lambda row: row["ms"])
                verdict = analyze(rows)
                self.assertEqual(verdict["diagnostic"], "INCOMPLETE_OR_UNSAFE_BENCH_TRACE")
                self.assertIn("EXTERNAL_SPAS_TRANSMISSION_PRESENT_IN_PASSIVE_PROBE", verdict["faults"])

    def test_panda_returned_bus_is_not_a_physical_tx_capture(self):
        # Pseudo-bus / host echo data cannot prove a physical ECU response
        # or a separate physical SPAS TX on bus 0.
        rows = traffic()
        rows.append(e(345, "tx", 0x16a, stock_frame(0x16a, 10), bus=128))
        rows.sort(key=lambda row: row["ms"])
        verdict = analyze(rows)
        self.assertNotIn("EXTERNAL_SPAS_TRANSMISSION_PRESENT_IN_PASSIVE_PROBE", verdict["faults"])
        self.assertEqual(verdict["diagnostic"], "ISOLATED_BENCH_HANDOFF_AND_RESTORE_OBSERVED")

    def test_bus_error_increment(self):
        rows = traffic()
        next(r for r in reversed(rows) if r["direction"] == "health")["errors"] = 1
        self.assertIn("CAN_ERROR_COUNTER_INCREASED:errors", analyze(rows)["faults"])

    def test_missing_health(self):
        rows = [r for r in traffic() if r["direction"] != "health"]
        self.assertIn("NO_INDEPENDENT_BEFORE_AFTER_CAN_HEALTH_MONITOR", analyze(rows)["faults"])

    def test_negative_ack(self):
        rows = traffic()
        next(r for r in rows if r["ms"] == 210 and r["address"] == "0x7b9")["data"] = "037f102200000000"
        self.assertIn("ECU_NEGATIVE_RESPONSE", analyze(rows)["faults"])

    def test_spas_bad_crc(self):
        rows = traffic()
        bad = next(r for r in rows if r["ms"] == 20 and r["address"] == "0x165")
        bad["data"] = "0000" + bad["data"][4:]
        self.assertIn("INVALID_STOCK_SPAS_CRC_OR_LENGTH", analyze(rows)["faults"])

    def test_timestamps_monotonic(self):
        rows = traffic()
        rows[5]["ms"] = 10
        self.assertIn("NONMONOTONIC_CAPTURE_TIMESTAMPS", analyze(rows)["faults"])

    def test_reply_not_sufficient_without_restore(self):
        rows = [r for r in traffic() if r["ms"] < 350]
        self.assertIn("ECU_RESTORE_NOT_ATTEMPTED", analyze(rows)["faults"])

    def test_missing_health_metrics_must_fail(self):
        rows = traffic()
        for row in rows:
            if row["direction"] == "health":
                row.pop("errors")
        result = analyze(rows)
        self.assertIn("INCOMPLETE_CAN_HEALTH_SAMPLE", result["faults"])
        self.assertNotEqual(result["diagnostic"], "ISOLATED_BENCH_HANDOFF_AND_RESTORE_OBSERVED")

    def test_missing_post_restore_health_must_fail(self):
        rows = [r for r in traffic() if r["direction"] != "health" or r["ms"] <= 150]
        self.assertIn("NO_HEALTH_SAMPLE_AFTER_RESTORE", analyze(rows)["faults"])

    def test_health_capture_gap_must_fail(self):
        rows = [r for r in traffic() if r["direction"] != "health" or r["ms"] in (0, 50, 100, 600, 650)]
        self.assertIn("HEALTH_MONITOR_COVERAGE_GAP", analyze(rows)["faults"])

    def test_no_health_during_oem_silence(self):
        rows = [r for r in traffic() if r["direction"] != "health" or not 240 <= r["ms"] <= 350]
        self.assertIn("NO_HEALTH_SAMPLE_DURING_OEM_SILENCE", analyze(rows)["faults"])

    def test_capture_dropped_fault(self):
        rows = traffic()
        next(r for r in rows if r["direction"] == "health" and r["ms"] == 300)["capture_dropped"] = 1
        self.assertIn("CAN_ERROR_COUNTER_INCREASED:capture_dropped", analyze(rows)["faults"])

    def test_counter_reset_fault(self):
        rows = traffic()
        for r in rows:
            if r["direction"] == "health":
                r["errors"] = 1
        next(r for r in reversed(rows) if r["direction"] == "health")["errors"] = 0
        self.assertIn("CAN_COUNTER_RESET_OR_UNTRUSTWORTHY:errors", analyze(rows)["faults"])

    def test_nan_timestamp_fault(self):
        rows = traffic()
        rows[2]["ms"] = float("nan")
        self.assertIn("INVALID_CAPTURE_TIMESTAMP", analyze(rows)["faults"])

    def test_bad_health_types_fault(self):
        rows = traffic()
        next(r for r in rows if r["direction"] == "health")["bus_off"] = "false"
        self.assertIn("INVALID_CAN_HEALTH_SAMPLE", analyze(rows)["faults"]

    def test_missing_bus_must_fail(self):
        rows = traffic()
        for r in rows:
            r.pop("bus", None)
        result = analyze(rows)
        self.assertIn("MISSING_OR_INVALID_CAN_BUS", result["faults"])
        self.assertNotEqual(result["diagnostic"], "ISOLATED_BENCH_HANDOFF_AND_RESTORE_OBSERVED")

    def test_missing_bus_only_from_ecu_ack_must_fail(self):
        rows = traffic()
        next(r for r in rows if r["address"] == "0x7b9").pop("bus")
        result = analyze(rows)
        self.assertIn("MISSING_OR_INVALID_CAN_BUS", result["faults"])
        self.assertIn("ECU_EXTENDED_SESSION_NOT_ACKNOWLEDGED", result["faults"])

    def test_negative_bus_must_fail(self):
        rows = traffic()
        rows[4]["bus"] = -1
        self.assertIn("MISSING_OR_INVALID_CAN_BUS", analyze(rows)["faults"])

    def test_boolean_bus_must_fail(self):
        rows = traffic()
        rows[4]["bus"] = False
        self.assertIn("MISSING_OR_INVALID_CAN_BUS", analyze(rows)["faults"])

    def test_malformed_can_data_must_fail(self):
        rows = traffic()
        next(r for r in rows if r["address"] == "0x7b9")["data"] = "NOT_HEX"
        self.assertIn("INVALID_CAN_ADDRESS_OR_DATA", analyze(rows)["faults"])

    def test_missing_address_must_fail(self):
        rows = traffic()
        next(r for r in rows if r["direction"] == "rx").pop("address")
        self.assertIn("INVALID_CAN_ADDRESS_OR_DATA", analyze(rows)["faults"])

    def test_malformed_event_does_not_crash(self):
        rows = traffic() + [None]
        self.assertIn("INVALID_CAPTURE_EVENT", analyze(rows)["faults"])

    def test_oem_counter_jump_detected(self):
        rows = traffic()
        r = next(r for r in rows if r["ms"] == 100 and r["address"] == "0x16a")
        r["data"] = stock_frame(0x16a, 12)
        self.assertIn("STOCK_COUNTER_DISCONTINUITY", analyze(rows)["faults"])

    def test_captured_diagnostic_requests_not_counted_as_verifier_tx(self):
        result = analyze(traffic())
        self.assertEqual(result["captured_aprk_requests"], 4)
        self.assertEqual(result["offline_verifier_can_tx"], 0)
        self.assertFalse(result["evidence_is_authenticated"])

    def test_no_factory_stream_during_session_must_fail(self):
        rows = [r for r in traffic() if not (r["address"] in ("0x165", "0x16a") and 190 < r["ms"] < 220)]
        verdict = analyze(rows)
        self.assertIn("OEM_SPAS_NOT_ACTIVE_IMMEDIATELY_BEFORE_DISABLE", verdict["faults"])
        self.assertNotEqual(verdict["diagnostic"], "ISOLATED_BENCH_HANDOFF_AND_RESTORE_OBSERVED")

    def test_only_one_factory_spas_during_session_must_fail(self):
        rows = [r for r in traffic() if not (r["address"] == "0x16a" and r["ms"] == 214)]
        self.assertIn("OEM_SPAS_NOT_ACTIVE_IMMEDIATELY_BEFORE_DISABLE", analyze(rows)["faults"])

    def test_unbounded_silence_period_must_fail(self):
        rows = traffic()
        for row in rows:
            if row["ms"] >= 350:
                row["ms"] += 2000
        rows.sort(key=lambda row: row["ms"])
        self.assertIn("UNBOUNDED_OEM_QUIET_INTERVAL", analyze(rows)["faults"])

    def test_good_trace_proves_pre_disable_stock_live(self):
        r = analyze(traffic())
        self.assertTrue(r["proof"]["stock_active_after_session_ack_before_disable"])

    def test_malformed_session_request_never_returns_not_tested(self):
        rows = traffic()
        req = next(r for r in rows if r["address"] == "0x7b1")
        req.pop("bus")
        result = analyze(rows)
        self.assertEqual(result["diagnostic"], "INCOMPLETE_OR_UNSAFE_BENCH_TRACE")
        self.assertIn("MISSING_OR_INVALID_CAN_BUS", result["faults"])

    def test_malformed_session_request_hex_never_returns_not_tested(self):
        rows = traffic()
        req = next(r for r in rows if r["address"] == "0x7b1")
        req["data"] = "XYZ"
        result = analyze(rows)
        self.assertEqual(result["diagnostic"], "INCOMPLETE_OR_UNSAFE_BENCH_TRACE")
        self.assertIn("INVALID_CAN_ADDRESS_OR_DATA", result["faults"])

    def test_bad_bus_in_capture_must_fail(self):
        rows = traffic()
        rows.append({"ms": 620, "direction": "health", "bus": "UNKNOWN"})
        rows.sort(key=lambda row: row["ms"])
        self.assertIn("MISSING_OR_INVALID_CAN_BUS", analyze(rows)["faults"])

    def test_duplicated_ecu_session_ack_not_accepted(self):
        rows = traffic()
        rows.append(e(216, "rx", 0x7b9, "0250030000000000"))
        rows.sort(key=lambda row: row["ms"])
        result = analyze(rows)
        self.assertIn("DUPLICATE_POSITIVE_ECU_RESPONSE:session", result["faults"])
        self.assertEqual(result["diagnostic"], "INCOMPLETE_OR_UNSAFE_BENCH_TRACE")

    def test_preexisting_bus_error_not_clean_bench(self):
        rows = traffic()
        for row in rows:
            if row["direction"] == "health":
                row["errors"] = 2
        self.assertIn("NONZERO_INITIAL_CAN_ERROR_OR_LOSS_COUNTER", analyze(rows)["faults"])

    def test_offline_tool_does_not_claim_vehicle_tx_audit(self):
        verdict = analyze(traffic())
        self.assertIsNone(verdict["vehicle_can_tx"])
        self.assertEqual(verdict["offline_verifier_can_tx"], 0)
        self.assertEqual(verdict["captured_aprk_requests"], 4)


if __name__ == "__main__":
    unittest.main()
