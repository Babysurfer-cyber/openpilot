"""Source-backed regression checks for Fleet/UI/navigation issues F5-F8, M2-M4, E1.

Does not compile Qt or run ffmpeg against real EV6 footage.
"""
from __future__ import annotations

import ast
import base64
import os
from pathlib import Path
import tempfile
import unittest
from typing import Any

ROOT = Path(__file__).resolve().parents[3]


def load_function(relpath, function_name, globals_dict):
  path = ROOT / relpath
  parsed = ast.parse(path.read_text(encoding="utf-8"))
  funcs = [node for node in parsed.body
           if isinstance(node, ast.FunctionDef) and node.name == function_name]
  assert len(funcs) == 1, (relpath, function_name)
  scope = dict(globals_dict)
  expr = ast.fix_missing_locations(ast.Module(body=[funcs[0]], type_ignores=[]))
  exec(compile(expr, str(path), "exec"), scope)
  return scope[function_name]


class MainAuditUiRegression(unittest.TestCase):
  def test_f8_complete_gif_accepted_and_truncated_gif_rejected(self):
    fn = load_function("selfdrive/frogpilot/fleetmanager/fleet_manager.py",
                       "_valid_preview_image", {"os": os})
    # Transparent 1x1 GIF (includes image data subblocks and trailer).
    valid = base64.b64decode("R0lGODlhAQABAIAAAAAAAP///ywAAAAAAQABAAACAUwAOw==")
    with tempfile.TemporaryDirectory() as directory:
      path = os.path.join(directory, "preview.gif")
      with open(path, "wb") as handle:
        handle.write(valid)
      self.assertTrue(fn(path))
      with open(path, "wb") as handle:
        handle.write(valid[:-1])
      self.assertFalse(fn(path))
      with open(path, "wb") as handle:
        handle.write(valid[:13] + b"x"*100)
      self.assertFalse(fn(path))

  def test_f5_onroad_view_is_selected_before_vipc_thread_wait(self):
    cc = (ROOT / "selfdrive/ui/qt/home.cc").read_text()
    section = cc.split("void HomeWindow::hideDriverCameraPreview()", 1)[1].split("void HomeWindow::showDriverView", 1)[0]
    self.assertLess(section.index("slayout->setCurrentWidget(onroad)"),
                    section.index("driver_camera_preview->stopVipcThread()"))
    self.assertIn("QTimer::singleShot(50", section)

  def test_f6_head_route_exits_before_ffmpeg_process_created(self):
    tree = ast.parse((ROOT / "selfdrive/frogpilot/fleetmanager/fleet_manager.py").read_text())
    func = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "fcamera")
    early_returns = [i for i, n in enumerate(func.body)
                     if isinstance(n, ast.If) and ast.unparse(n.test) == "request.method == 'HEAD'"]
    processes = [i for i, n in enumerate(func.body)
                 if isinstance(n, ast.Assign)
                 and any(isinstance(t, ast.Name) and t.id == "process" for t in n.targets)]
    self.assertEqual(len(early_returns), 1)
    self.assertEqual(len(processes), 1)
    self.assertLess(early_returns[0], processes[0])

  def test_f7_camera_switch_links_follow_active_segment(self):
    html = (ROOT / "selfdrive/frogpilot/fleetmanager/templates/route.html").read_text()
    for source in ("qcamera", "fcamera", "dcamera", "ecamera"):
      self.assertIn('data-camera-switch="' + source + '"', html)
    self.assertIn("document.querySelectorAll('[data-camera-switch]')", html)
    self.assertIn("encodeURIComponent(number)", html)

  def test_m3_late_info_response_must_match_current_clip(self):
    html = (ROOT / "selfdrive/frogpilot/fleetmanager/templates/route.html").read_text()
    self.assertIn("const requestedSegment = currentSegment;", html)
    self.assertIn("if (requestedSegment !== currentSegment) return;", html)

  def test_m4_fleet_layout_has_no_debug_note(self):
    html = (ROOT / "selfdrive/frogpilot/fleetmanager/templates/layout.html").read_text()
    self.assertNotIn("remove cookie toggler", html)

  def test_m2_atc2_has_same_blink_condition_as_atc(self):
    cc = (ROOT / "selfdrive/ui/carrot.cc").read_text()
    self.assertIn('desired_src.compare("atc", Qt::CaseInsensitive) == 0', cc)
    self.assertIn('desired_src.compare("atc2", Qt::CaseInsensitive) == 0', cc)

  def test_e1_lone_surrogate_is_scrubbed_before_cereal(self):
    fn = load_function("selfdrive/carrot/carrot_navi_cereal.py", "_text", {"Any": Any})
    cleaned = fn("\ud800route name", 96)
    self.assertEqual(cleaned, "\ufffdroute name")
    cleaned.encode("utf-8", errors="strict")
    self.assertEqual(fn("clean road", 5), "clean")


if __name__ == "__main__":
  unittest.main()
