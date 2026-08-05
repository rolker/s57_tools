# enc_updater

Cron-friendly NOAA ENC chart updater
([unh_marine_autonomy ADR-0010 D7](../../unh_marine_autonomy/docs/decisions/0010-geospatial-world-model.md)
— the uma project ADR, not the workspace repo's ADR-0010): keeps an ENC corpus
current and regenerates the bathymetry store's `chart` layer **wholesale** on
change. Offline tooling — it runs wherever imports run (dev machines, the
boat), never inside the ROS runtime.

## Update cycle

1. **Download** — fetch the [NOAA ENC product catalog](https://charts.noaa.gov/ENCs/ENCProdCat.xml)
   and compare each configured cell's edition/update against the corpus
   manifest (`<corpus_dir>/.manifest.json`). Changed cells are downloaded and
   validated — byte count against the catalog's `zipfile_size` plus a full
   zip CRC pass (the catalog publishes no checksums) — before anything is
   extracted. A failed or invalid download leaves the corpus and manifest
   untouched.
2. **Change detection** — no changed cells *and* the active layer's edition
   registry matches the corpus → exit 0 quietly (idempotent; `--force`
   overrides). A corpus *ahead of* the active layer (e.g. a prior run's swap
   was refused by the interlock) also triggers regeneration.
3. **Regenerate** — run `s57_to_geotiff` over the whole corpus, then
   `import_geotiff --stage` each exported GeoTIFF at the GGGS level the
   exporter chose (parsed from its log, always passed explicitly). Staging
   happens **adjacent to the store** (`<store parent>/.enc_updater_staging.<pid>`)
   because the commit's atomic rename requires the same filesystem.
4. **Sanity check** — at least one staged tile, none empty, and a band-1
   (ellipsoidal height) spot check against `depth_range`. A corrupt or
   truncated regeneration can never swap in.
5. **Swap** — the edition registry (`editions.json`) is written *inside the
   staged `chart/` dir*, then `import_geotiff --commit` atomically swaps
   tiles + registry into the store as one rename.
6. **Nav-down interlock (enforced)** — immediately before the swap, the
   updater probes the ROS graph (`ros2 node list`) and **refuses to commit**
   if any configured nav-stack node is present. With nodes configured, a
   failed probe also refuses (fail closed). Exit code 2 marks a refusal;
   the next cron slot retries.
7. **Health surfacing** — `<corpus_dir>/.updater_health.json` records
   `last_download_attempt`, `last_download_ok`, `last_regen_ok` and
   `last_error`, so repeated silent cron failures age the layer loudly.

## Usage

```bash
enc_updater --config region.yaml            # one full update cycle
enc_updater --config region.yaml --dry-run  # download + export + stage + validate,
                                            # no interlock probe, no swap
enc_updater --config region.yaml --force    # regenerate even without changes
```

Exit codes: `0` success or no-op, `1` failure (previous layer intact),
`2` interlock refusal (previous layer intact).

Run it from a shell with the workspace sourced — the `s57_to_geotiff` and
`import_geotiff` CLIs are resolved through the ament index (`lib/<pkg>/`),
and the interlock's `ros2` CLI comes from the same environment (or set
`nav_liveness.ros_setup` to source one explicitly).

### Cron example

```cron
0 2 * * * bash -lc 'source ~/project11/.agent/scripts/setup.bash && enc_updater --config /etc/enc_updater/region.yaml'
```

## Configuration

See [`config/region_example.yaml`](config/region_example.yaml) (New Castle /
Isles of Shoals cells) for the full annotated schema: corpus/store paths,
cell list, vertical-datum grids for the export, `depth_range` sanity bounds
(the default upper bound +100 m admits lake surfaces), tool-path overrides,
and subprocess timeouts.

**Nav-liveness contract**: `nav_liveness.nodes` lists the exact node names
(as printed by `ros2 node list`, e.g. `/bizzy/controller`) whose presence
refuses the swap. An empty/omitted list disables the probe entirely — only
appropriate on hosts that never run navigation (dev machines). There is no
sentinel file or side channel: the probe reads the live ROS graph at swap
time only.

**Probe environment alignment (critical)**: the probe runs `ros2 node list`
in the updater's own environment. If that environment's `ROS_DOMAIN_ID` or
`RMW_IMPLEMENTATION` differs from the live navigation stack's, the probe
queries the *wrong* DDS graph, sees no nodes, and the interlock **fails open**
— the swap proceeds while nav is active. Fail-closed only covers probe
*errors*; a successful-but-blind empty result is indistinguishable from "nav
genuinely down". A cron job is the likely offender: it inherits none of an
interactive shell's ROS env. So:

- Pin `nav_liveness.ros_domain_id` to the nav stack's domain (0–232). The
  updater exports it into the probe's environment, overriding whatever the
  cron/sourced env carried.
- Source the same setup the nav stack uses via `nav_liveness.ros_setup`.
- Ensure `RMW_IMPLEMENTATION` matches the nav stack's — export it in the cron
  environment or the sourced setup (there is no config key for it; DDS
  discovery only sees peers on the same middleware).

## Deployment prerequisite (uma#276)

The store's chart layer only *feeds a costmap safely* on hosts whose
`bathymetry_layer` build includes
[unh_marine_autonomy#276](https://github.com/rolker/unh_marine_autonomy/issues/276)
(worst-case-clearance cost model + confidence gate, merged 2026-08-03).
Pre-#276 builds render high-uncertainty chart cells LETHAL under
`unsurveyed_is_lethal`. Rebuild such hosts before pointing this updater at a
store that a live costmap consumes.

## Troubleshooting

- **`work dir already exists`** — a previous run may still be active (or
  died hard). Verify no `enc_updater` process is running before removing the
  named `.enc_updater_staging.*` / `.enc_updater_export.*` directory.
- **`runs must not overlap`** — a concurrent `enc_updater` holds the store
  lock (`.enc_updater.lock` beside the store); the second run refuses rather
  than racing on the staged layer. Space cron entries so a slow regeneration
  can't overlap the next slot. The lock file persists between runs (only the
  advisory lock is released); do not delete it while a run is active.
- **`configured cell(s) not in catalog`** — config typo, or NOAA withdrew
  the cell; fix the config either way.
- **Exit 2 every night** — navigation genuinely up at the cron hour, the
  probe failing closed (check `ros2 node list` by hand), or a stale node
  name in `nav_liveness.nodes`.
- **Layer aging** — compare `.updater_health.json`'s `last_regen_ok` with
  `last_download_ok`: downloads succeeding while regeneration fails points
  at the export/stage/sanity steps (`last_error.phase` says which).
