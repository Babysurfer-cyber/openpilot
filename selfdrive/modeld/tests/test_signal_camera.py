"""C3 reference-image conversion and camera-ray consistency."""
import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.selfdrive.carrot.signal_camera import REFERENCE_SIZE, reference_focal, reference_transform
from openpilot.selfdrive.modeld.signal_tracking_shadow import copy_nv12_rgb


@pytest.mark.parametrize('sensor', ['ar0231', 'ox03c10', 'os04c10'])
def test_reference_camera_preserves_physical_rays_and_image_centre(sensor):
  camera = DEVICE_CAMERAS['tici', sensor].fcam
  scale, offset_x, offset_y = reference_transform(*camera.size)
  focal = reference_focal(camera)
  # Several calibrated rays must remain the same after resizing/cropping.
  for ray_x, ray_y in ((0., 0.), (.1, -.1), (-.2, .05)):
    pixel_x = camera.width / 2 + camera.focal_length * ray_x
    pixel_y = camera.height / 2 + camera.focal_length * ray_y
    reference_x, reference_y = pixel_x * scale + offset_x, pixel_y * scale + offset_y
    assert (reference_x - REFERENCE_SIZE[0] / 2) / focal == pytest.approx(ray_x)
    assert (reference_y - REFERENCE_SIZE[1] / 2) / focal == pytest.approx(ray_y)
    assert (reference_x-offset_x)/scale == pytest.approx(pixel_x)
    assert (reference_y-offset_y)/scale == pytest.approx(pixel_y)


def test_c3_padded_nv12_reference_is_centred_owned_and_does_not_mutate_capture():
  width, height, stride = 1928, 1208, 1984
  offset = stride * height + 128
  raw = np.full(offset + stride * height // 2, 77, np.uint8)
  y = raw[:stride*height].reshape(height, stride)
  uv = raw[offset:].reshape(height//2, stride)
  y[:, :width] = 16
  uv[:, :width] = 128
  # A bright 20-pixel square centred in the source must remain centred.
  y[594:614, 954:974] = 235
  before = raw.copy()
  frame = SimpleNamespace(width=width, height=height, stride=stride, uv_offset=offset, data=raw)
  rgb = copy_nv12_rgb(frame)
  assert rgb.shape == (760, 1344, 3)
  ys, xs = np.where(rgb[:, :, 0] > 200)
  assert abs(xs.mean() - 672) < 1 and abs(ys.mean() - 380) < 1
  assert (rgb[370:385, 662:677].max(axis=2) > 0).any()
  np.testing.assert_array_equal(raw, before)
  old = rgb.copy()
  raw[:] = 0
  np.testing.assert_array_equal(rgb, old)


@pytest.mark.parametrize('size', [(1164, 874), (1920, 1080), (0, 0), (1344, 1208)])
def test_unknown_camera_geometries_are_rejected(size):
  with pytest.raises(ValueError, match='geometry'):
    reference_transform(*size)


@pytest.mark.parametrize('started,mode,expected', [(False, 1, False), (True, 0, False),
                                                (True, 1, True), (True, 2, True),
                                                (True, 3, False), (True, -1, False)])
def test_manager_only_starts_observer_onroad_with_selected_mode(started, mode, expected):
  # Execute the production gate without importing native process/IPC machinery.
  path = Path(__file__).parents[3] / 'system/manager/process_config.py'
  source = ast.parse(path.read_text())
  node = next(n for n in source.body if isinstance(n, ast.FunctionDef) and n.name == 'signal_color_observer')
  namespace = dict(Params=object, car=SimpleNamespace(CarParams=object))
  exec(compile(ast.Module([node], type_ignores=[]), str(path), 'exec'), namespace)
  params = SimpleNamespace(get_int=lambda key: mode)
  assert namespace['signal_color_observer'](started, params, None) is expected
