#!/usr/bin/env python3
"""Offline source contracts for EV6 0x3C1 panda opt-in; no CAN or car imports."""
import ast
import unittest
from pathlib import Path

HYUNDAI = Path(__file__).resolve().parents[2] / "car" / "hyundai"


def _source(path):
  return (HYUNDAI / path).read_text(encoding="utf-8")


class TestEV6ExplicitPandaOptIn(unittest.TestCase):

  def test_panda_tx_flag_is_not_enabled_for_all_ev6_at_startup(self):
    tree = ast.parse(_source("interface.py"))
    marker = "HyundaiSafetyFlags.EV6_BLINKER_STALK_TEST"
    assignments = [
      node for node in ast.walk(tree)
      if isinstance(node, ast.AugAssign) and marker in ast.unparse(node.value)
    ]
    self.assertEqual(len(assignments), 1, "exactly one EV6 experimental safety flag assignment")
    assignment = assignments[0]
    gating_ifs = [
      node for node in ast.walk(tree)
      if isinstance(node, ast.If) and assignment in list(ast.walk(ast.Module(body=node.body, type_ignores=[])))
    ]
    self.assertTrue(gating_ifs, "experimental safety flag must be inside an if guard")
    tests = [ast.unparse(node.test) for node in gating_ifs]
    self.assertTrue(any(
      "not docs" in t and "candidate == CAR.KIA_EV6" in t and
      'params.get_int(\'Ev6BlinkerTest\') in (1, 2)' in t for t in tests
    ), "only a specifically selected left/right one-shot may arm the panda flag")

  def test_one_shot_request_is_consumed_before_vehicle_state_checks(self):
    tree = ast.parse(_source("carcontroller.py"))
    methods = [
      node for node in ast.walk(tree)
      if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
      and node.name == "_ev6_blinker_stalk_trial"
    ]
    self.assertEqual(len(methods), 1)
    method = methods[0]
    consumes = [
      node for node in ast.walk(method)
      if isinstance(node, ast.Call) and ast.unparse(node.func) == "params.put_int"
      and len(node.args) >= 2 and isinstance(node.args[0], ast.Constant)
      and node.args[0].value == "Ev6BlinkerTest" and isinstance(node.args[1], ast.Constant)
      and node.args[1].value == 0
    ]
    self.assertEqual(len(consumes), 1)
    parked = [
      node for node in ast.walk(method)
      if isinstance(node, ast.Assign) and any(
        isinstance(t, ast.Name) and t.id == "parked" for t in node.targets
      )
    ]
    self.assertEqual(len(parked), 1)
    self.assertLess(consumes[0].lineno, parked[0].lineno,
                    "failed one-shots must not wait for a later P/brake state")
    guard = ast.unparse(parked[0].value)
    self.assertTrue(all(f"not CC.{k}" in guard for k in ("enabled", "latActive", "longActive")),
                    "host trial must reject any active openpilot control")


  def test_compiled_panda_policy_must_deny_stalk_while_controls_allowed(self):
    safety = Path(__file__).resolve().parents[1] / "safety" / "safety_hyundai_canfd.h"
    source = safety.read_text(encoding="utf-8")
    needle = "bool allowed = hyundai_ev6_blinker_stalk_test"
    start = source.find(needle)
    self.assertNotEqual(start, -1)
    end = source.find("if (allowed)", start)
    self.assertGreater(end, start)
    self.assertIn("!controls_allowed", source[start:end],
                  "panda must independently deny any experimental stalk TX during control")


if __name__ == "__main__":
  unittest.main()
