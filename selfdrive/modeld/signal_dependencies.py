"""Install the optional CPU observer's single missing dependency in isolation."""
import importlib
from pathlib import Path
import subprocess
import sys

OPENCV_PACKAGE = 'opencv-python-headless==4.11.0.86'
INSTALL_TIMEOUT = 45
OBSERVATION_FILES = (
  '/dev/shm/carrot_signal_observation.json',
  '/dev/shm/carrot_signal_display.json',
  '/dev/shm/carrot_signal_start_observation.json',
  '/dev/shm/carrot_signal_prepare_observation.json',
)


def ensure_opencv(target=None):
  """Import OpenCV, or install only its binary wheel outside the system Python.

  Called by the enabled, low-priority observer. It never runs in modeld, the
  planner or manager, and pip is forbidden from changing NumPy/dependencies.
  """
  if target is None:
    from openpilot.common.basedir import BASEDIR
    target = Path(BASEDIR) / 'pydeps'
  target = Path(target)
  if str(target) not in sys.path:
    sys.path.insert(0, str(target))
  try:
    return importlib.import_module('cv2')
  except ImportError:
    pass
  target.mkdir(parents=True, exist_ok=True)
  subprocess.run(
    [sys.executable, '-m', 'pip', 'install', '--disable-pip-version-check',
     '--no-input', '--no-deps', '--only-binary=:all:', '--retries', '0', '--timeout', '15',
     '--target', str(target), '--upgrade', OPENCV_PACKAGE],
    timeout=INSTALL_TIMEOUT, check=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
  )
  importlib.invalidate_caches()
  return importlib.import_module('cv2')


def clear_observations(paths=OBSERVATION_FILES):
  for path in paths:
    try:
      Path(path).unlink(missing_ok=True)
    except OSError:
      pass
