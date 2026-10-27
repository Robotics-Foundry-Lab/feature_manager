"""Monitor and control the feature containers through docker and docker compose."""

from concurrent.futures import ThreadPoolExecutor
import copy
from datetime import datetime, timezone
import json
import logging
import os
from pathlib import Path
import re
import subprocess
import threading
import time
from typing import Any, Callable, Dict, Iterator, List, Optional

import docker
from docker.errors import DockerException, NotFound

from .config import Config, Feature

LOG = logging.getLogger('feature_manager.controller')

BOOT_ID_FILE = Path('/proc/sys/kernel/random/boot_id')
ANSI_ESCAPE = re.compile(r'\x1b\[[0-9;?]*[A-Za-z]')


def _parse_docker_time(value: str) -> Optional[float]:
    """Convert docker's RFC3339 timestamps (with nanoseconds) to epoch seconds."""
    if not value or value.startswith('0001-'):
        return None
    try:
        dt = datetime.strptime(value[:19], '%Y-%m-%dT%H:%M:%S')
    except ValueError:
        return None
    return dt.replace(tzinfo=timezone.utc).timestamp()


def _cpu_percent(stats: Dict[str, Any]) -> Optional[float]:
    cpu, pre = stats.get('cpu_stats') or {}, stats.get('precpu_stats') or {}
    try:
        cpu_delta = cpu['cpu_usage']['total_usage'] - pre['cpu_usage']['total_usage']
        system_delta = cpu['system_cpu_usage'] - pre['system_cpu_usage']
    except KeyError:
        return None
    cpus = cpu.get('online_cpus') or len(cpu['cpu_usage'].get('percpu_usage') or [1])
    if system_delta <= 0 or cpu_delta < 0:
        return 0.0
    return round(cpu_delta / system_delta * cpus * 100.0, 1)


def _memory_bytes(stats: Dict[str, Any]) -> Optional[int]:
    mem = stats.get('memory_stats') or {}
    if 'usage' not in mem:
        return None
    details = mem.get('stats') or {}
    # Same as `docker stats`: subtract the page cache (cgroup v2: inactive_file, v1: cache)
    cache = details.get('inactive_file',
                        details.get('total_inactive_file', details.get('cache', 0)))
    return max(mem['usage'] - cache, 0)


class FeatureController:
    """Owns all docker access; the web server and the ROS node are thin adapters on top."""

    def __init__(self, config: Config, state_file: Path,
                 status_interval: float = 2.0, stats_interval: float = 5.0):
        self.config = config
        self.state_file = Path(state_file)
        self.status_interval = status_interval
        self.stats_interval = stats_interval

        self._local = threading.local()
        self._locks = {fid: threading.Lock() for fid in config.features}
        self._runtime: Dict[str, Dict[str, Any]] = {
            fid: {'pending': None, 'last_error': None, 'last_action': None}
            for fid in config.features
        }
        self._stats: Dict[str, Dict[str, Any]] = {}
        self._snapshot: Dict[str, Any] = {'timestamp': 0, 'features': {}}
        self._snapshot_lock = threading.Lock()
        self._refresh_now = threading.Event()
        self._stop = threading.Event()
        self._listeners: List[Callable[[Dict[str, Any]], None]] = []
        self.version = 0

    # ------------------------------------------------------------------ docker helpers

    @property
    def client(self) -> docker.DockerClient:
        # docker-py clients are not meant to be shared between threads
        if not hasattr(self._local, 'client'):
            self._local.client = docker.from_env()
        return self._local.client

    def _container(self, feature: Feature):
        try:
            return self.client.containers.get(feature.container)
        except NotFound:
            return None

    def _compose(self, feature: Feature, *args: str, env: Optional[Dict[str, str]] = None):
        if not (feature.compose_dir / 'docker-compose.yml').exists() and \
                not (feature.compose_dir / 'docker-compose.yaml').exists():
            raise FileNotFoundError(f'No docker-compose.yml in {feature.compose_dir}')
        cmd = ['docker', 'compose', *args, feature.service]
        run_env = dict(os.environ)
        # The other features live in the same compose project; they are not orphans of this one
        run_env['COMPOSE_IGNORE_ORPHANS'] = 'true'
        run_env.update(env or {})
        LOG.info('%s: %s %s', feature.id, ' '.join(f'{k}={v}' for k, v in (env or {}).items()),
                 ' '.join(cmd))
        result = subprocess.run(cmd, cwd=feature.compose_dir, env=run_env,
                                capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(result.stderr.strip()[-2000:] or f'exit code {result.returncode}')

    # ------------------------------------------------------------------ status

    def feature_status(self, feature: Feature) -> Dict[str, Any]:
        status: Dict[str, Any] = {
            'state': 'not created', 'running': False, 'started_at': None, 'finished_at': None,
            'exit_code': None, 'restart_count': 0, 'health': None, 'active_options': {},
            'image': None, 'error': None,
        }
        try:
            container = self._container(feature)
        except DockerException as exc:
            status.update(state='unknown', error=str(exc))
            return status

        if container is not None:
            attrs = container.attrs
            state = attrs.get('State') or {}
            env = dict(item.split('=', 1) for item in (attrs.get('Config') or {}).get('Env') or []
                       if '=' in item)
            active = {}
            for opt in feature.options:
                if opt.key in env:
                    try:
                        active[opt.key] = opt.normalize(env[opt.key])
                    except ValueError:
                        active[opt.key] = env[opt.key]
            status.update(
                state=state.get('Status', 'unknown'),
                running=bool(state.get('Running')),
                started_at=_parse_docker_time(state.get('StartedAt', '')),
                finished_at=_parse_docker_time(state.get('FinishedAt', '')),
                exit_code=state.get('ExitCode'),
                restart_count=attrs.get('RestartCount', 0),
                health=(state.get('Health') or {}).get('Status'),
                active_options=active,
                image=(attrs.get('Config') or {}).get('Image'),
            )
        status.update(self._runtime[feature.id])
        stats = self._stats.get(feature.id) if status['running'] else None
        status['cpu_percent'] = stats.get('cpu_percent') if stats else None
        status['mem_bytes'] = stats.get('mem_bytes') if stats else None
        status['mem_limit'] = stats.get('mem_limit') if stats else None
        return status

    def refresh(self) -> Dict[str, Any]:
        snapshot = {
            'timestamp': time.time(),
            'features': {fid: self.feature_status(f) for fid, f in self.config.features.items()},
        }
        with self._snapshot_lock:
            changed = _comparable(snapshot) != _comparable(self._snapshot)
            self._snapshot = snapshot
            if changed:
                self.version += 1
        if changed:
            for listener in list(self._listeners):
                try:
                    listener(copy.deepcopy(snapshot))
                except Exception:  # noqa: B902 - a broken listener must not stop monitoring
                    LOG.exception('Status listener failed')
        return snapshot

    def snapshot(self) -> Dict[str, Any]:
        with self._snapshot_lock:
            return copy.deepcopy(self._snapshot)

    def add_listener(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        """Call `callback(snapshot)` whenever the status of any feature changes."""
        self._listeners.append(callback)

    def _sample_stats(self, feature: Feature) -> Optional[Dict[str, Any]]:
        try:
            container = self._container(feature)
            if container is None or container.status != 'running':
                return None
            stats = container.stats(stream=False)
        except DockerException:
            return None
        return {
            'cpu_percent': _cpu_percent(stats),
            'mem_bytes': _memory_bytes(stats),
            'mem_limit': (stats.get('memory_stats') or {}).get('limit'),
        }

    def _status_loop(self):
        while not self._stop.is_set():
            try:
                self.refresh()
            except Exception:  # noqa: B902 - keep monitoring through docker hiccups
                LOG.exception('Status refresh failed')
            self._refresh_now.wait(self.status_interval)
            self._refresh_now.clear()

    def _stats_loop(self):
        # One docker stats call takes 1-2 s, so all features are sampled in parallel
        with ThreadPoolExecutor(max_workers=len(self.config.features) or 1) as pool:
            while not self._stop.is_set():
                features = list(self.config.features.values())
                results = pool.map(self._sample_stats, features)
                self._stats = {f.id: s for f, s in zip(features, results) if s}
                self._stop.wait(self.stats_interval)

    def start_background(self):
        self.refresh()
        threading.Thread(target=self._status_loop, name='status', daemon=True).start()
        threading.Thread(target=self._stats_loop, name='stats', daemon=True).start()

    def shutdown(self):
        self._stop.set()
        self._refresh_now.set()

    # ------------------------------------------------------------------ actions

    def request(self, feature_id: str, action: str,
                options: Optional[Dict[str, Any]] = None) -> None:
        """Validate and queue start/stop/restart; the compose call runs in a worker thread."""
        feature = self.config.find(feature_id)
        if feature is None:
            raise KeyError(f'Unknown feature: {feature_id}')
        if action not in ('start', 'stop', 'restart', 'toggle'):
            raise ValueError(f'Unknown action: {action}')
        if action == 'toggle':
            action = 'stop' if self.feature_status(feature)['running'] else 'start'
        resolved = feature.resolve_options(options) if action == 'start' else None

        lock = self._locks[feature.id]
        if not lock.acquire(blocking=False):
            pending = self._runtime[feature.id]['pending']
            raise RuntimeError(f'{feature.display_name} is busy ({pending})')
        self._runtime[feature.id].update(pending={'start': 'starting', 'stop': 'stopping',
                                                  'restart': 'restarting'}[action],
                                         last_error=None)
        self._refresh_now.set()
        threading.Thread(target=self._run_action, args=(feature, action, resolved, lock),
                         name=f'{action}-{feature.id}', daemon=True).start()

    def _run_action(self, feature: Feature, action: str, options, lock: threading.Lock):
        try:
            if action == 'start':
                # Compose recreates the container only if the options (env) changed
                self._compose(feature, 'up', '-d', env=feature.option_env(options))
            elif action == 'stop':
                self._compose(feature, 'stop')
            else:
                self._compose(feature, 'restart')
            self._runtime[feature.id]['last_action'] = {
                'action': action, 'options': options, 'time': time.time(), 'ok': True}
            LOG.info('%s: %s done', feature.id, action)
        except Exception as exc:  # noqa: B902 - report every failure to the UI
            LOG.error('%s: %s failed: %s', feature.id, action, exc)
            self._runtime[feature.id]['last_error'] = str(exc)
            self._runtime[feature.id]['last_action'] = {
                'action': action, 'options': options, 'time': time.time(), 'ok': False}
        finally:
            self._runtime[feature.id]['pending'] = None
            lock.release()
            self._refresh_now.set()

    # ------------------------------------------------------------------ logs

    def logs(self, feature_id: str, tail: int = 200) -> List[str]:
        feature = self.config.find(feature_id)
        container = self._container(feature) if feature else None
        if container is None:
            return []
        raw = container.logs(tail=tail, timestamps=False).decode('utf-8', errors='replace')
        return [ANSI_ESCAPE.sub('', line) for line in raw.splitlines()]

    def follow_logs(self, feature_id: str, tail: int = 200):
        """Return (stream, line iterator). Close the stream to stop following."""
        feature = self.config.find(feature_id)
        container = self._container(feature) if feature else None
        if container is None:
            return None, iter(())
        stream = container.logs(stream=True, follow=True, tail=tail)

        def lines() -> Iterator[str]:
            buffer = ''
            for chunk in stream:
                buffer += chunk.decode('utf-8', errors='replace')
                *complete, buffer = buffer.split('\n')
                for line in complete:
                    yield ANSI_ESCAPE.sub('', line.rstrip('\r'))
            if buffer:
                yield ANSI_ESCAPE.sub('', buffer)
        return stream, lines()

    # ------------------------------------------------------------------ boot

    def _load_state(self) -> Dict[str, Any]:
        try:
            return json.loads(self.state_file.read_text())
        except (OSError, ValueError):
            return {}

    def _save_state(self, state: Dict[str, Any]) -> None:
        try:
            self.state_file.parent.mkdir(parents=True, exist_ok=True)
            self.state_file.write_text(json.dumps(state, indent=2))
        except OSError as exc:
            LOG.warning('Could not write %s: %s', self.state_file, exc)

    def autostart_once(self) -> List[str]:
        """Start the autostart features once per host boot (not on every restart of this node)."""
        try:
            boot_id = BOOT_ID_FILE.read_text().strip()
        except OSError:
            boot_id = None
        state = self._load_state()
        if boot_id and state.get('boot_id') == boot_id:
            LOG.info('Autostart already done for this boot, skipping')
            return []

        started = []
        for feature in self.config.features.values():
            if not feature.autostart or self.feature_status(feature)['running']:
                continue
            try:
                self.request(feature.id, 'start')
                started.append(feature.id)
            except Exception as exc:  # noqa: B902 - one broken feature must not block the rest
                LOG.error('Autostart of %s failed: %s', feature.id, exc)
        LOG.info('Autostart: %s', ', '.join(started) or 'nothing to start')
        state['boot_id'] = boot_id
        self._save_state(state)
        return started


def _comparable(snapshot: Dict[str, Any]) -> Dict[str, Any]:
    """Snapshot without the timestamp, used to detect real changes."""
    return {k: v for k, v in snapshot.items() if k != 'timestamp'}
