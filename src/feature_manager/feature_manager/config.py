"""Load the feature registry (features.yml) into typed objects."""

from dataclasses import dataclass, field
import os
from pathlib import Path
import re
from typing import Any, Dict, List, Optional

import yaml

_ENV_PATTERN = re.compile(r'\$\{([A-Za-z_][A-Za-z0-9_]*)(?::-([^}]*))?\}')

TRUE_STRINGS = ('1', 'true', 'yes', 'on')


def expand_env(value: str) -> str:
    """Expand ${VAR} and ${VAR:-default} like docker compose does."""
    return _ENV_PATTERN.sub(lambda m: os.environ.get(m.group(1)) or (m.group(2) or ''), value)


@dataclass
class Choice:
    value: str
    description: str = ''
    # Features whose running state makes this choice a bad idea (shown as a warning in the UI)
    warn_if_running: List[str] = field(default_factory=list)


@dataclass
class Option:
    key: str  # environment variable passed to docker compose
    label: str
    type: str  # 'bool' or 'choice'
    default: Any
    description: str = ''
    choices: List[Choice] = field(default_factory=list)

    def normalize(self, value: Any) -> Any:
        """Convert a value from the web/ROS/container env into the option's native type."""
        if self.type == 'bool':
            if isinstance(value, bool):
                return value
            return str(value).strip().lower() in TRUE_STRINGS
        value = str(value).strip()
        if value not in [c.value for c in self.choices]:
            raise ValueError(f'{self.key}: {value!r} is not one of '
                             f'{", ".join(c.value for c in self.choices)}')
        return value

    def to_env(self, value: Any) -> str:
        value = self.normalize(value)
        if self.type == 'bool':
            return 'true' if value else 'false'
        return value


@dataclass
class Feature:
    id: str
    display_name: str
    description: str
    folder: str
    container: str
    service: str
    compose_dir: Path
    autostart: bool = False
    requires: List[str] = field(default_factory=list)
    options: List[Option] = field(default_factory=list)

    def option(self, key: str) -> Optional[Option]:
        return next((o for o in self.options if o.key == key), None)

    def defaults(self) -> Dict[str, Any]:
        return {o.key: o.default for o in self.options}

    def resolve_options(self, requested: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        """Merge requested options over the defaults and validate them."""
        resolved = self.defaults()
        for key, value in (requested or {}).items():
            opt = self.option(key)
            if opt is None:
                raise ValueError(f'{self.id} has no option {key!r}')
            resolved[key] = opt.normalize(value)
        return resolved

    def option_env(self, options: Dict[str, Any]) -> Dict[str, str]:
        return {o.key: o.to_env(options[o.key]) for o in self.options}

    def to_dict(self) -> Dict[str, Any]:
        return {
            'id': self.id,
            'display_name': self.display_name,
            'description': self.description,
            'container': self.container,
            'autostart': self.autostart,
            'requires': self.requires,
            'options': [{
                'key': o.key,
                'label': o.label,
                'type': o.type,
                'default': o.default,
                'description': o.description,
                'choices': [c.__dict__ for c in o.choices],
            } for o in self.options],
        }


@dataclass
class Config:
    workspace_root: Path
    features: Dict[str, Feature]

    def find(self, name: str) -> Optional[Feature]:
        """Look a feature up by id or container name."""
        if name in self.features:
            return self.features[name]
        return next((f for f in self.features.values() if f.container == name), None)


def _parse_option(key: str, raw: Dict[str, Any]) -> Option:
    opt_type = raw.get('type', 'bool')
    choices = []
    for value, info in (raw.get('choices') or {}).items():
        if not isinstance(info, dict):
            info = {'description': info or ''}
        choices.append(Choice(str(value), ' '.join(str(info.get('description', '')).split()),
                              list(info.get('warn_if_running') or [])))
    if opt_type not in ('bool', 'choice'):
        raise ValueError(f'Option {key}: unknown type {opt_type!r}')
    if opt_type == 'choice' and not choices:
        raise ValueError(f'Option {key}: choice options need a choices list')

    opt = Option(key=key, label=raw.get('label', key), type=opt_type, default=None,
                 description=' '.join(str(raw.get('description', '')).split()),
                 choices=choices)
    default = raw.get('default', False if opt_type == 'bool' else choices[0].value)
    opt.default = opt.normalize(default)
    return opt


def load_config(path: Path) -> Config:
    with open(path, 'r', encoding='utf-8') as fh:
        raw = yaml.safe_load(fh) or {}

    workspace_root = Path(expand_env(str(raw.get('workspace_root', '${WORKSPACE_ROOT}'))))
    features = {}
    for fid, info in (raw.get('features') or {}).items():
        info = info or {}
        folder = info.get('folder', fid)
        container = info.get('container', fid)
        features[fid] = Feature(
            id=fid,
            display_name=info.get('display_name', fid),
            description=' '.join(str(info.get('description', '')).split()),
            folder=folder,
            container=container,
            # All feature compose files live in a folder called .devcontainer, so they share the
            # compose project name "devcontainer". Every compose call is scoped to this service.
            service=info.get('service', container),
            compose_dir=workspace_root / folder / info.get('compose_dir', '.devcontainer'),
            autostart=bool(info.get('autostart', False)),
            requires=list(info.get('requires') or []),
            options=[_parse_option(k, v or {}) for k, v in (info.get('options') or {}).items()],
        )
    return Config(workspace_root=workspace_root, features=features)
