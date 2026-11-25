"""Regression coverage for Fleet Manager's seekable one-minute stream wrapper.

The browser controls request a fresh remuxed stream at the selected timestamp,
rather than relying on unsupported HTTP byte ranges in a live ffmpeg pipe.
"""

from types import SimpleNamespace

import pytest

from openpilot.selfdrive.frogpilot.fleetmanager import helpers


@pytest.mark.parametrize(("filename", "position"), (
  ("/tmp/0--0/qcamera.ts", 0.0),
  ("/tmp/0--0/qcamera.ts", 10.0),
  ("/tmp/0--0/qcamera.ts", 35.5),
  ("/tmp/0--0/fcamera.hevc", 20.0),
))
def test_ffmpeg_remux_starts_at_requested_position(monkeypatch, filename, position):
  commands = []

  def fake_popen(command, **kwargs):
    commands.append((command, kwargs))
    return SimpleNamespace(stdout=None)

  monkeypatch.setattr(helpers.subprocess, "Popen", fake_popen)
  helpers.ffmpeg_mp4_wrap_process_builder(filename, start_seconds=position)
  assert len(commands) == 1
  command, _ = commands[0]

  assert command[command.index("-i") + 1] == filename
  assert command[command.index("-c") + 1] == "copy"
  assert command[command.index("-f") + 1] == "mp4"
  assert "empty_moov" in command
  if position:
    assert command[command.index("-ss") + 1] == f"{position:.3f}"
    assert command.index("-ss") > command.index("-i")
  else:
    assert "-ss" not in command
  if filename.endswith(".hevc"):
    assert command[command.index("-f") + 1] == "hevc"
    assert command[command.index("-vtag") + 1] == "hvc1"
