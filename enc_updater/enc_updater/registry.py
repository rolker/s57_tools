"""
Edition registry and corpus manifest I/O.

Both files share one schema::

    {"generated_at": "<ISO-8601 UTC>", "cells": {"US5NH02M": {"edition": 25, "update": 3}}}

- The **corpus manifest** (``<corpus_dir>/.manifest.json``) records what the
  on-disk ENC corpus holds.
- The **edition registry** (``editions.json``) records what the *chart layer*
  was built from. It is written inside the staged ``chart/`` directory before
  ``import_geotiff --commit`` — ``--commit`` (``replaceChartLayer``) renames the
  whole staged ``chart/`` dir into the store with one atomic ``rename(2)``, so
  every file in it — ``.tif`` tiles and ``editions.json`` alike — rides along in
  a single commit. (``replaceChartLayer`` only *validates* ``.tif`` tiles,
  ignoring non-``.tif`` entries when it checks the staged layer is non-empty and
  well-named; it does not filter them out of the swap.) Placing the registry in
  ``chart/`` therefore makes the rename the single commit point for tiles and
  registry together.
"""

import datetime
import json
import os
import tempfile
from typing import Dict

from . import UpdaterError

REGISTRY_NAME = 'editions.json'
MANIFEST_NAME = '.manifest.json'


def _utc_now_iso() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds')


def load_cells(path: str) -> Dict[str, dict]:
    """Return the ``cells`` mapping from a registry/manifest file; {} if absent."""
    if not os.path.exists(path):
        return {}
    try:
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)
    except (OSError, ValueError) as e:
        raise UpdaterError(f'registry: cannot read {path}: {e}')
    cells = data.get('cells') if isinstance(data, dict) else None
    if not isinstance(cells, dict):
        raise UpdaterError(f'registry: {path} has no "cells" mapping')
    return cells


def write_cells(path: str, cells: Dict[str, dict]) -> None:
    """Atomically write a registry/manifest file (temp file + rename)."""
    payload = {'generated_at': _utc_now_iso(), 'cells': cells}
    directory = os.path.dirname(path) or '.'
    fd, tmp = tempfile.mkstemp(prefix='.registry.', dir=directory)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            json.dump(payload, f, indent=2, sort_keys=True)
            f.write('\n')
        os.replace(tmp, path)
    except OSError as e:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise UpdaterError(f'registry: cannot write {path}: {e}')


def manifest_path(corpus_dir: str) -> str:
    """Path of the corpus manifest inside a corpus directory."""
    return os.path.join(corpus_dir, MANIFEST_NAME)


def active_registry_path(store_dir: str) -> str:
    """Path of the active chart layer's edition registry inside a store."""
    return os.path.join(store_dir, 'chart', REGISTRY_NAME)


def staged_registry_path(staged_chart_dir: str) -> str:
    """Path of the edition registry inside a staged ``chart/`` directory."""
    return os.path.join(staged_chart_dir, REGISTRY_NAME)
