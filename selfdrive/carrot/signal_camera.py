"""Reference image geometry shared by the CPU observer, planner and displays.

The tracker uses 1344 x 760 coordinates. Comma 3 road frames are uniformly
scaled to that width and cropped symmetrically vertically. Camera intrinsics
must use the same scale; the image centre remains (672, 380).
"""

REFERENCE_SIZE = (1344, 760)
SUPPORTED_SIZES = ((1344, 760), (1928, 1208))


def reference_transform(width, height):
  """Return (scale, offset_x, offset_y) for reference = source * scale + offset."""
  if (width, height) not in SUPPORTED_SIZES:
    raise ValueError('unsupported signal camera geometry')
  scale = REFERENCE_SIZE[0] / width
  return scale, 0., (REFERENCE_SIZE[1] - height * scale) / 2.


def reference_focal(camera):
  """Scale the known camera focal length to the observer reference image."""
  scale, _, _ = reference_transform(*camera.size)
  return camera.focal_length * scale


def reference_image(rgb):
  import cv2
  import numpy as np
  height, width = rgb.shape[:2]
  scale, offset_x, offset_y = reference_transform(width, height)
  if (width, height) == REFERENCE_SIZE:
    return rgb
  transform = np.array(((scale, 0., offset_x), (0., scale, offset_y)), dtype=np.float64)
  return cv2.warpAffine(rgb, transform, REFERENCE_SIZE, flags=cv2.INTER_LINEAR)
