"""CPU traffic-signal observer entrypoint and offline color-comparison helpers.

SignalAssistMode selects the source branch's production CPU tracker. The legacy
learned color scanner remains an offline comparison helper, never the runtime
recognition backend and never a replacement for the driving model.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time

DIRECTORY = Path('/data/signal-color-shadow')
MODEL_SHA256 = 'c091fad2b08d9a59e0e061413ec62ec181a77f371791d6f78e6a5bd61b148d06'
REFERENCE_SIZE = (1344, 760)
CLASSES = ('background', 'red', 'green')
MAX_AGE_MS = 1500


def requested(directory=DIRECTORY):
  from openpilot.selfdrive.carrot.signal_mode import flag_override
  override = flag_override(Path(directory) / 'enabled')
  if override is not None:
    return override
  try:
    return (Path(directory) / 'enabled').read_text().strip() == '1'
  except OSError:
    return False


def validate_artifact(directory=DIRECTORY):
  directory = Path(directory)
  meta = json.loads((directory / 'installed.json').read_text())
  if (meta.get('version') != 1 or meta.get('mode') != 'camera_comparison_only' or
      meta.get('model_sha256') != MODEL_SHA256 or meta.get('classes') != list(CLASSES)):
    raise ValueError('invalid signal-color comparison manifest')
  path = directory / 'signal_color_pool.onnx'
  if hashlib.sha256(path.read_bytes()).hexdigest() != MODEL_SHA256:
    raise ValueError('signal-color ONNX checksum mismatch')
  return path, meta


def window_boxes():
  width, height = REFERENCE_SIZE
  return [(x, y, x + w, y + h) for w, h in ((96, 48), (144, 72), (216, 108))
          for y in range(0, int(height * .6) - h + 1, h // 2)
          for x in range(int(width * .05), int(width * .95) - w + 1, w // 2)]


def rgb_from_nv12(data, width, height, stride, uv_offset):
  """Copy active padded NV12 planes, BT.601 limited-range -> RGB.

  This is independent of modeld warp/calibration and does not mutate IPC data.
  Raw camera versus encoded-video color differences require separate validation.
  """
  import numpy as np
  from PIL import Image
  if (width <= 0 or height <= 0 or width % 2 or height % 2 or stride < width or
      stride % 2 or uv_offset < stride * height):
    raise ValueError('invalid NV12 dimensions')
  raw = np.frombuffer(data, np.uint8)
  end = uv_offset + stride * (height // 2)
  if raw.size < end:
    raise ValueError('truncated NV12 buffer')
  y = raw[:height * stride].reshape(height, stride)[:, :width]
  uv = raw[uv_offset:end].reshape(height // 2, stride)[:, :width]
  u = uv[:, 0::2].repeat(2, axis=0).repeat(2, axis=1)
  v = uv[:, 1::2].repeat(2, axis=0).repeat(2, axis=1)
  # Use Pillow's native matrix loop rather than allocating many full-resolution
  # int32 intermediates on the little CPU cores. RGB here is just the 3-byte
  # input container; its channels hold Y, U, V until the matrix is applied.
  packed = Image.fromarray(np.stack((y, u, v), axis=2))
  matrix = (298 / 256, 0, 409 / 256, -(298 * 16 + 409 * 128) / 256,
            298 / 256, -100 / 256, -208 / 256, (-298 * 16 + 308 * 128) / 256,
            298 / 256, 516 / 256, 0, -(298 * 16 + 516 * 128) / 256)
  return np.asarray(packed.convert('RGB', matrix))


def select_candidates(probabilities, boxes, count=3):
  import numpy as np
  p = np.asarray(probabilities)
  if (p.shape != (len(boxes), 3) or not np.isfinite(p).all() or (p < 0).any() or
      (p > 1).any() or not np.allclose(p.sum(axis=1), 1, atol=1e-4)):
    raise ValueError('malformed color probabilities')
  scores = p[:, 1:].max(axis=1)
  order = np.argsort(-scores, kind='stable')
  selected = []
  for index in order:
    box = boxes[int(index)]
    # Preserve the strongest-window decision. Suppress overlapping runners-up
    # only in the diagnostic top-three list, never in the primary result.
    overlaps = False
    for old in selected:
      other = old['box_xyxy']
      area = max(0, min(box[2], other[2]) - max(box[0], other[0])) * max(0, min(box[3], other[3]) - max(box[1], other[1]))
      union = (box[2] - box[0]) * (box[3] - box[1]) + (other[2] - other[0]) * (other[3] - other[1]) - area
      overlaps |= area / union > .3
    if overlaps:
      continue
    color = CLASSES[1 + int(p[index, 1:].argmax())]
    selected.append({'box_xyxy': list(box), 'color': color, 'score': float(scores[index]),
                     'probabilities': p[index].astype(float).tolist()})
    if len(selected) >= count:
      break
  return selected


def scan_image(rgb_image, session):
  import numpy as np
  from PIL import Image
  image = Image.fromarray(rgb_image).resize(REFERENCE_SIZE, Image.Resampling.BILINEAR)
  boxes = window_boxes()
  predictions = []
  for start in range(0, len(boxes), 32):
    group = boxes[start:start + 32]
    # Transfer all resized patches to NumPy once per batch. Each patch is still
    # resized independently, preserving the recorded-image experiment exactly.
    atlas = Image.new('RGB', (96, 32 * len(group)))
    for index, box in enumerate(group):
      atlas.paste(image.crop(box).resize((96, 32), Image.Resampling.BILINEAR), (0, index * 32))
    batch = np.asarray(atlas, np.float32).reshape(len(group), 32, 96, 3).transpose(0, 3, 1, 2) / 255
    predictions.append(session.run(None, {'rgb': batch})[0])
  return select_candidates(np.concatenate(predictions), boxes)


def result_fields(candidates, age_ms):
  if not candidates or age_ms < 0 or age_ms > MAX_AGE_MS:
    return {'prediction': 'unknown', 'usable': False, 'reason': 'stale_frame'}
  strongest = candidates[0]
  accepted = strongest['score'] >= .8
  return {'prediction': strongest['color'] if accepted else 'unknown', 'usable': accepted,
          'reason': 'score_threshold' if not accepted else 'comparison_only_no_lane_association'}


def next_delay(work_wall, work_cpu):
  # At most 1 Hz, with <=25% of one CPU average process duty after startup.
  # A slow process never catches up on queued frames.
  return max(.05, 1.0 - work_wall, 3 * work_cpu)


def configure_worker_scheduling():
  # The manager's spawn launcher can create logging/IPC threads before main().
  # Linux affinity/nice are per-thread; include those existing threads too.
  allowed = os.sched_getaffinity(0) & {0, 1, 2, 3}
  if not allowed:
    raise RuntimeError('no permitted little CPU for signal observer')
  for task in Path('/proc/self/task').iterdir():
    try:
      tid = int(task.name)
      os.sched_setaffinity(tid, allowed)
      os.sched_setscheduler(tid, os.SCHED_OTHER, os.sched_param(0))
      os.setpriority(os.PRIO_PROCESS, tid, 19)
    except ProcessLookupError:
      pass


def run(directory=DIRECTORY, duration=None):
  # The original branch's production fallback is its lightweight CPU tracker.
  # This C3 port deliberately has no neural backend or private model dependency.
  from openpilot.selfdrive.modeld.signal_tracking_shadow import tracking_requested, run as run_tracking
  if tracking_requested(directory):
    return run_tracking(directory, duration)


def prepare_runtime():
  from openpilot.selfdrive.modeld.signal_dependencies import ensure_opencv, clear_observations
  from openpilot.common.swaglog import cloudlog
  clear_observations()
  # A missing wheel/network never stalls the driving model or creates a rapid
  # manager restart loop. Three bounded attempts, then wait until disabled.
  for delay in (0, 30, 120):
    until = time.monotonic() + delay
    while time.monotonic() < until:
      if not requested():
        return False
      time.sleep(max(0., min(1., until-time.monotonic())))
    if not requested():
      return False
    try:
      configure_worker_scheduling()
      ensure_opencv()
      return True
    except Exception:
      clear_observations()
      cloudlog.exception('signalObserverDependencyUnavailable')
  return False


def main():
  # Manager calls main without CLI arguments. Latch ordinary failures until a
  # disable/ignition restart; manager otherwise re-launches exited processes.
  try:
    if prepare_runtime():
      run()
  except Exception:
    from openpilot.common.swaglog import cloudlog
    cloudlog.exception('signalColorShadowDisabled; original control unchanged')
  from openpilot.selfdrive.modeld.signal_dependencies import clear_observations
  clear_observations()
  while requested():
    time.sleep(1)


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Read-only live camera signal comparison')
  parser.add_argument('mode', choices=('on', 'off', 'status', 'run'))
  parser.add_argument('--seconds', type=float)
  args = parser.parse_args()
  if args.mode in ('on', 'off'):
    if args.mode == 'on':
      validate_artifact()
    DIRECTORY.mkdir(parents=True, exist_ok=True)
    temp = DIRECTORY / 'enabled.tmp'
    temp.write_text('1' if args.mode == 'on' else '0')
    temp.replace(DIRECTORY / 'enabled')
  if args.mode == 'run':
    run(duration=args.seconds)
  else:
    print(json.dumps({'requested': requested(), 'control': 'original',
                      'latest': json.loads((DIRECTORY / 'latest.json').read_text()) if (DIRECTORY / 'latest.json').exists() else None}))
