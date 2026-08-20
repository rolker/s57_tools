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
import os
import sys
from typing import List, Optional

from . import config as config_mod
from . import datum_provisioner
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
                             'prune the corpus, probe the interlock, or swap '
                             '(would-be prunes are printed)')
    parser.add_argument('--force', action='store_true',
                        help='regenerate even if change detection sees no change')
    args = parser.parse_args(argv)

    try:
        cfg = config_mod.load_config(args.config)
    except UpdaterError as e:
        print(f'enc_updater: {e}', file=sys.stderr)
        return 1

    # Bootstrap the store dir up front (#39): the commit's replaceChartLayer
    # swaps into an existing store and refuses a missing one — which used to
    # surface only at the very last step, after minutes of download/export
    # work. Deliberately os.mkdir, NOT makedirs: only the leaf is created,
    # so an unmounted data volume or mistyped path (missing parents) still
    # fails loudly here instead of silently forking the whole pipeline onto
    # a shadow store on the wrong filesystem. A store_dir path occupied by
    # a plain file also fails here, with its own message — folding it into
    # the mkdir failure would blame a "missing parent" for what is really a
    # config error, sending the operator debugging the wrong thing.
    if os.path.exists(cfg.store_dir) and not os.path.isdir(cfg.store_dir):
        print(f'enc_updater: store_dir {cfg.store_dir} exists but is not a '
              'directory — fix the config (or remove the file)',
              file=sys.stderr)
        return 1
    if not os.path.isdir(cfg.store_dir):
        try:
            os.mkdir(cfg.store_dir)
        except OSError as e:
            print(f'enc_updater: cannot create store dir {cfg.store_dir}: {e}'
                  ' — parent must already exist (is the data volume mounted?)',
                  file=sys.stderr)
            return 1
        print(f'enc_updater: created store dir {cfg.store_dir}')

    # Provision the vertical-datum grids the D7 export needs before touching
    # the corpus: absent grids would only surface as an export failure later,
    # and the fetch records its own health error on failure.
    try:
        datum_provisioner.ensure_geoid(cfg)
        datum_provisioner.ensure_vdatum(cfg)
    except UpdaterError as e:
        print(f'enc_updater: {e}', file=sys.stderr)
        return 1

    health.record_download_attempt(cfg.corpus_dir)
    # The pre-update manifest keys are the previous cycle's cell set; after a
    # real update_corpus run (which prunes deselected cells) the manifest
    # holds exactly the current set, so the diff below surfaces membership
    # changes — a NOAA rescheme in region mode, a config edit in cells mode.
    # The load sits inside the try: a corrupt manifest is a download-phase
    # failure like any other (clean exit 1 + health record, no traceback).
    try:
        previous = set(
            registry.load_cells(registry.manifest_path(cfg.corpus_dir)))
        changed, manifest = downloader.update_corpus(cfg, dry_run=args.dry_run)
    except UpdaterError as e:
        print(f'enc_updater: {e}', file=sys.stderr)
        health.record_error(cfg.corpus_dir, 'download', str(e))
        return 1
    health.record_download_ok(cfg.corpus_dir)
    # Not in dry-run: pruning is skipped there, so the manifest still carries
    # deselected cells (removals would be silently missed) and a preview run
    # must not write a last_selection_change record — the would-prune lines
    # from prune_corpus already show the membership changes a real run would
    # make.
    if not args.dry_run:
        current = set(manifest)
        if current != previous:
            added, removed = current - previous, previous - current
            print('enc_updater: cell set changed: '
                  f'+{sorted(added)} -{sorted(removed)}')
            health.record_selection_change(
                cfg.corpus_dir, current, added, removed)

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
