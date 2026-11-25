"""Fleet Manager camera defaults: prefer fcamera when present.

The lightweight Dashcam thumbnail still comes from qcamera.ts; the playback
selection must be independent of that preview source.
"""

import pytest

from openpilot.selfdrive.frogpilot.fleetmanager import helpers as fleet


ROUTE = "00000d3f--5a12c700c9"
S0 = f"{ROUTE}--0"
S1 = f"{ROUTE}--1"


def recording(tmp_path, segment, filename, data=b"recorded"):
  folder = tmp_path / segment
  folder.mkdir(exist_ok=True)
  (folder / filename).write_bytes(data)


def test_fcamera_preferred_when_both_exist(tmp_path):
  recording(tmp_path, S0, "qcamera.ts")
  recording(tmp_path, S1, "fcamera.hevc")
  camera, playable = fleet.preferred_camera_segments([S0, S1], str(tmp_path))
  assert (camera, playable) == ("fcamera", [S1])
  # The qcamera thumbnail path remains independently available.
  assert fleet.playable_camera_segments([S0, S1], "qcamera", str(tmp_path)) == [S0]


def test_missing_fcamera_falls_back_to_qcamera(tmp_path):
  recording(tmp_path, S0, "qcamera.ts")
  assert fleet.preferred_camera_segments([S0, S1], str(tmp_path)) == ("qcamera", [S0])


def test_fcamera_only_still_opens_view_without_thumbnail(tmp_path):
  recording(tmp_path, S1, "fcamera.hevc")
  assert fleet.preferred_camera_segments([S0, S1], str(tmp_path)) == ("fcamera", [S1])
  assert fleet.playable_camera_segments([S0, S1], "qcamera", str(tmp_path)) == []


def test_zero_byte_camera_file_is_not_treated_as_recording(tmp_path):
  recording(tmp_path, S0, "fcamera.hevc", b"")
  recording(tmp_path, S0, "qcamera.ts")
  assert fleet.preferred_camera_segments([S0], str(tmp_path)) == ("qcamera", [S0])


def test_empty_route_has_no_default_playable_video(tmp_path):
  assert fleet.preferred_camera_segments([S0, S1], str(tmp_path)) == ("qcamera", [])


@pytest.mark.parametrize("camera, filename", [
  ("qcamera", "qcamera.ts"),
  ("fcamera", "fcamera.hevc"),
  ("dcamera", "dcamera.hevc"),
  ("ecamera", "ecamera.hevc"),
])
def test_explicit_camera_selection_is_respected(tmp_path, camera, filename):
  recording(tmp_path, S0, filename)
  assert fleet.playable_camera_segments([S0], camera, str(tmp_path)) == [S0]
  assert fleet.playable_camera_segments([S0], "unknown", str(tmp_path)) == []
