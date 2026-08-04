"""
CLI entry point: ``enc_updater --config region.yaml [--dry-run] [--force]``.

Exit codes:

* 0 — success, including the idempotent no-op ("nothing changed upstream").
* 1 — a failure (download, export, staging, sanity, commit, config). The
  previous chart layer and registry are intact.
* 2 — interlock refusal: navigation was active (or the probe failed closed).
  The staged work is discarded; the previous layer stands. Cron will simply
  try again on its next slot.
"""

import argparse
import sys
from typing import List, Optional

from . import config as config_mod
from . import downloader
from . import health
from . import InterlockRefusal, UpdaterError
from . import regenerator
from . import registry


def _need_regeneration(cfg, changed: List[str], manifest, force: bool) -> Optional[str]:
    """
    Return the reason a regeneration is needed, or None for a no-op.

    Besides fresh downloads, a corpus that is *ahead of the active layer*
    also triggers regeneration — that is the state a previous run leaves
    behind when its swap was refused by the interlock.
    """
    if force:
        return '--force'
    if changed:
        return f'{len(changed)} cell(s) updated'
    active = registry.load_cells(registry.active_registry_path(cfg.store_dir))
    if active != manifest:
        return 'active chart layer differs from corpus manifest'
    return None


def main(argv: Optional[List[str]] = None) -> int:
    """Run one update cycle. See the module docstring for exit codes."""
    parser = argparse.ArgumentParser(
        prog='enc_updater',
        description='Keep an ENC corpus current and regenerate the bathymetry '
                    "store's chart layer wholesale on change (ADR-0010 D7).")
    parser.add_argument('--config', required=True, help='region config YAML')
    parser.add_argument('--dry-run', action='store_true',
                        help='download, export, stage and validate, but do not '
                             'probe the interlock or swap')
    parser.add_argument('--force', action='store_true',
                        help='regenerate even if change detection sees no change')
    args = parser.parse_args(argv)

    try:
        cfg = config_mod.load_config(args.config)
    except UpdaterError as e:
        print(f'enc_updater: {e}', file=sys.stderr)
        return 1

    health.record_download_attempt(cfg.corpus_dir)
    try:
        changed, manifest = downloader.update_corpus(cfg)
    except UpdaterError as e:
        print(f'enc_updater: {e}', file=sys.stderr)
        health.record_error(cfg.corpus_dir, 'download', str(e))
        return 1
    health.record_download_ok(cfg.corpus_dir)

    reason = _need_regeneration(cfg, changed, manifest, args.force)
    if reason is None:
        print('enc_updater: no upstream change — nothing to do')
        return 0
    print(f'enc_updater: regenerating chart layer ({reason})')

    try:
        regenerator.regenerate(cfg, manifest, dry_run=args.dry_run)
    except InterlockRefusal as e:
        print(f'enc_updater: {e}', file=sys.stderr)
        health.record_error(cfg.corpus_dir, 'interlock', str(e))
        return 2
    except UpdaterError as e:
        print(f'enc_updater: {e}', file=sys.stderr)
        health.record_error(cfg.corpus_dir, 'regenerate', str(e))
        return 1
    if not args.dry_run:
        health.record_regen_ok(cfg.corpus_dir)
    return 0


if __name__ == '__main__':
    sys.exit(main())
