"""
Updater health surfacing.

``<corpus_dir>/.updater_health.json`` records the last download attempt, the
last successful download, the last successful regeneration, and the last
error. Cron runs are silent by nature; this file is how repeated failures age
the chart layer *loudly* — a monitoring check (or a human) can compare
``last_regen_ok`` against the calendar.
"""

import datetime
import json
import os
import tempfile

HEALTH_NAME = '.updater_health.json'


def _utc_now_iso() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds')


def _path(corpus_dir: str) -> str:
    return os.path.join(corpus_dir, HEALTH_NAME)


def _record(corpus_dir: str, **fields) -> None:
    """
    Merge fields into the health file.

    Best-effort: health writes never abort an update run, but a failure is
    printed rather than swallowed.
    """
    path = _path(corpus_dir)
    data = {}
    try:
        if os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as f:
                loaded = json.load(f)
            if isinstance(loaded, dict):
                data = loaded
    except (OSError, ValueError) as e:
        print(f'enc_updater: warning: unreadable health file {path}: {e}')
    data.update(fields)
    try:
        os.makedirs(corpus_dir, exist_ok=True)
        fd, tmp = tempfile.mkstemp(prefix='.health.', dir=corpus_dir)
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, sort_keys=True)
            f.write('\n')
        os.replace(tmp, path)
    except OSError as e:
        print(f'enc_updater: warning: cannot write health file {path}: {e}')


def record_download_attempt(corpus_dir: str) -> None:
    """Record that a catalog fetch / download cycle started."""
    _record(corpus_dir, last_download_attempt=_utc_now_iso())


def record_download_ok(corpus_dir: str) -> None:
    """Record a successful download/change-detection cycle."""
    _record(corpus_dir, last_download_ok=_utc_now_iso())


def record_regen_ok(corpus_dir: str) -> None:
    """Record a successful regeneration + swap, and clear the last error."""
    _record(corpus_dir, last_regen_ok=_utc_now_iso(), last_error=None)


def record_error(corpus_dir: str, phase: str, message: str) -> None:
    """Record a failure with its phase; kept until the next successful swap."""
    _record(corpus_dir, last_error={
        'when': _utc_now_iso(), 'phase': phase, 'message': message,
    })
