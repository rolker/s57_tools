"""
Cron-friendly NOAA ENC chart updater (ADR-0010 D7, s57_tools#28).

ADR-0010 here is the unh_marine_autonomy project ADR
(docs/decisions/0010-geospatial-world-model.md in that repo), not the
workspace repo's ADR-0010.
"""


class UpdaterError(RuntimeError):
    """
    A failure that must leave the previous chart layer and registry intact.

    Every code path that raises this guarantees the live store, the corpus,
    and the active edition registry are exactly as they were before the
    failing step ran.
    """


class InterlockRefusal(UpdaterError):
    """Swap refused: navigation liveness detected, or the probe failed closed."""
