"""Regression tests for the laneless-only / LFA long-press ATC switch.

The configured ATC mode is not the ON/OFF state: LFA must never change the
driver's saved 0..3 mode, and holding LFA must generate only one toggle.
"""
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.car.cruise import ButtonType, VCruiseCarrot
from openpilot.selfdrive.carrot.carrot_serv import CarrotServ


class FakeParams:
  def __init__(self, mode=2, enabled=True):
    self.values = {"AutoTurnControl": mode, "AutoTurnControlEnabled": enabled}
    self.writes = []

  def get_int(self, key):
    return self.values.get(key, 0)

  def get_float(self, key):
    return float(self.get_int(key))

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def get(self, key, encoding=None):
    return "30" if key == "AutoCurveSpeedLowerLimit" else None

  def put_bool_nonblocking(self, key, value):
    self.values[key] = value
    self.writes.append((key, value))


@pytest.mark.parametrize("mode", [0, 1, 2, 3])
def test_long_lfa_toggles_atc_but_preserves_configured_mode(mode):
  ctrl = VCruiseCarrot.__new__(VCruiseCarrot)
  ctrl.params = FakeParams(mode=mode, enabled=True)
  ctrl._atc_toggle_enabled = True
  logs = []
  ctrl._add_log = logs.append

  ctrl._toggle_atc_enabled()
  assert ctrl._atc_toggle_enabled is False
  assert ctrl.params.values["AutoTurnControl"] == mode
  assert ctrl.params.writes == [("AutoTurnControlEnabled", False)]
  assert logs[-1] == "ATC OFF"

  ctrl._toggle_atc_enabled()
  assert ctrl._atc_toggle_enabled is True
  assert ctrl.params.values["AutoTurnControl"] == mode
  assert ctrl.params.writes[-1] == ("AutoTurnControlEnabled", True)
  assert logs[-1] == ("ATC ON" if mode > 0 else "ATC OFF")


@pytest.mark.parametrize("mode", [0, 1, 2, 3])
@pytest.mark.parametrize("enabled", [False, True])
def test_effective_atc_mode_matches_saved_zero_behavior(mode, enabled):
  svc = CarrotServ.__new__(CarrotServ)
  svc.params = FakeParams(mode=mode, enabled=enabled)
  svc.update_params()
  assert svc.autoTurnControl == (mode if enabled else 0)
  assert svc.params.values["AutoTurnControl"] == mode


def test_lfa_held_fires_once_without_short_press_on_release():
  ctrl = VCruiseCarrot.__new__(VCruiseCarrot)
  ctrl.button_cnt = 0
  ctrl.button_prev = ButtonType.unknown
  ctrl.long_pressed = False
  ctrl._cruise_button_long_delay = 40
  ctrl.button_long_time = 50
  ctrl._cruise_speed_unit_basic = 1
  ctrl._cruise_speed_unit = 10
  ctrl._cruise_button_mode = 0
  ctrl.is_metric = True

  pressed = SimpleNamespace(type=ButtonType.lfaButton, pressed=True)
  released = SimpleNamespace(type=ButtonType.lfaButton, pressed=False)

  _, event, long_pressed = ctrl._prepare_buttons(SimpleNamespace(buttonEvents=[pressed]), 80.)
  assert not event and not long_pressed

  emitted = []
  for _ in range(100):
    _, event, long_pressed = ctrl._prepare_buttons(SimpleNamespace(buttonEvents=[]), 80.)
    if event:
      emitted.append((event, long_pressed))
  assert emitted == [(ButtonType.lfaButton, True)]

  _, event, long_pressed = ctrl._prepare_buttons(SimpleNamespace(buttonEvents=[released]), 80.)
  assert not event and not long_pressed
