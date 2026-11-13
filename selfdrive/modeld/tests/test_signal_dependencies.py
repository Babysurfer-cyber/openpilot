from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.modeld import signal_dependencies as deps
from openpilot.selfdrive.modeld import signal_color_shadow as worker


def test_installed_opencv_never_invokes_package_manager(monkeypatch, tmp_path):
  module = object()
  monkeypatch.setattr(deps.importlib, 'import_module', lambda name: module)
  monkeypatch.setattr(deps.subprocess, 'run', lambda *a, **kw: pytest.fail('unneeded install'))
  assert deps.ensure_opencv(tmp_path) is module


def test_first_use_installs_only_pinned_binary_in_isolated_target(monkeypatch, tmp_path):
  imports, commands = [], []
  module = object()
  def load(name):
    imports.append(name)
    if len(imports) == 1:
      raise ModuleNotFoundError('cv2')
    return module
  monkeypatch.setattr(deps.importlib, 'import_module', load)
  monkeypatch.setattr(deps.importlib, 'invalidate_caches', lambda: None)
  monkeypatch.setattr(deps.subprocess, 'run', lambda args, **kwargs: commands.append((args, kwargs)))
  assert deps.ensure_opencv(tmp_path/'pydeps') is module
  args, kwargs = commands[0]
  assert args[:3] == [deps.sys.executable, '-m', 'pip']
  assert '--no-deps' in args and '--only-binary=:all:' in args
  assert args[args.index('--target')+1] == str(tmp_path/'pydeps')
  assert args[-1] == 'opencv-python-headless==4.11.0.86'
  assert kwargs['timeout'] <= 60 and kwargs['check']
  assert imports == ['cv2', 'cv2']


def test_install_timeout_propagates_without_unbounded_retry(monkeypatch, tmp_path):
  monkeypatch.setattr(deps.importlib, 'import_module', lambda name: (_ for _ in ()).throw(ImportError('missing')))
  def timeout(args, **kwargs):
    raise subprocess.TimeoutExpired(args, kwargs['timeout'])
  monkeypatch.setattr(deps.subprocess, 'run', timeout)
  with pytest.raises(subprocess.TimeoutExpired):
    deps.ensure_opencv(tmp_path)


def test_failure_clears_old_transports_and_has_bounded_backoff(monkeypatch, tmp_path):
  files = [tmp_path/name for name in ('start.json', 'prepare.json', 'display.json')]
  for path in files:
    path.write_text('stale')
  clock, attempts, errors = [0.], [], []
  def fail():
    attempts.append(clock[0])
    raise ImportError('not available')
  monkeypatch.setattr(worker, 'requested', lambda: True)
  monkeypatch.setattr(worker, 'configure_worker_scheduling', lambda: None)
  monkeypatch.setattr(worker.time, 'monotonic', lambda: clock[0])
  monkeypatch.setattr(worker.time, 'sleep', lambda delay: clock.__setitem__(0, clock[0]+delay))
  monkeypatch.setattr(deps, 'ensure_opencv', fail)
  clear = deps.clear_observations
  monkeypatch.setattr(deps, 'clear_observations', lambda: clear(files))
  monkeypatch.setitem(deps.sys.modules, 'openpilot.common.swaglog', SimpleNamespace(
    cloudlog=SimpleNamespace(exception=errors.append)))
  assert not worker.prepare_runtime()
  assert attempts == [0., 30., 150.] and len(errors) == 3
  assert not any(path.exists() for path in files)


def test_mode_zero_does_not_install_or_wait(monkeypatch):
  monkeypatch.setattr(worker, 'requested', lambda: False)
  monkeypatch.setattr(deps, 'ensure_opencv', lambda: pytest.fail('mode zero install'))
  monkeypatch.setattr(deps, 'clear_observations', lambda: None)
  monkeypatch.setattr(worker.time, 'sleep', lambda _: pytest.fail('mode zero wait'))
  assert not worker.prepare_runtime()
