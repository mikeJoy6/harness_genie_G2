"""Discover installed planner modules, isolating unavailable optional planners."""
from dataclasses import dataclass
import importlib
import pkgutil
import re
from typing import Type

from ._types import Planner as Algorithm


@dataclass(frozen=True)
class AlgorithmSpec:
    name: str
    description: str
    factory: Type[Algorithm]


_REGISTRY = {}
_LOADED = set()
_ERRORS = {}


def register(name, description):
    def decorate(cls):
        if not re.fullmatch(r'[a-z][a-z0-9_]*', name):
            raise ValueError(f'Invalid algorithm name: {name}')
        if name in _REGISTRY:
            raise ValueError(f'Duplicate algorithm name: {name}')
        if not issubclass(cls, Algorithm) or bool(getattr(cls, '__abstractmethods__', None)):
            raise TypeError('Planner must inherit Planner and implement plan(request)')
        _REGISTRY[name] = AlgorithmSpec(name, description, cls)
        return cls
    return decorate


def discover():
    package = importlib.import_module(__package__)
    for module in sorted(pkgutil.iter_modules(package.__path__), key=lambda item: item.name):
        if module.name.startswith('_') or module.name in _LOADED:
            continue
        before = dict(_REGISTRY)
        try:
            importlib.import_module(f'{__package__}.{module.name}')
        except Exception as error:
            _REGISTRY.clear()
            _REGISTRY.update(before)
            _ERRORS[module.name] = f'{type(error).__name__}: {error}'
        _LOADED.add(module.name)
    return dict(sorted(_REGISTRY.items())), dict(_ERRORS)
