# Test data: an S-57 exchange-set catalogue

`enc_root/` is laid out like an ENC_ROOT. It holds two files, both copied
unchanged from NOAA's `US5PSMCC.zip` (the harbour-scale cell for Portsmouth
Harbor, NH), downloaded on 2026-11-24 from
https://charts.noaa.gov/ENCs/US5PSMCC.zip (index:
https://charts.noaa.gov/ENCs/ENCs.shtml):

| File | Bytes | sha256 |
|------|-------|--------|
| `enc_root/CATALOG.031` | 1347 | `10a94bcfcff9a9aaa4f913304e6a100546579a6c2bf4654911f93b389440da8f` |
| `enc_root/USERAGREEMENT.TXT` | 5134 | `27b503af0727216809c8cc9cd73a8895073f43535f4e1bdba8b0b14c599f848f` |

`CATALOG.031` is the exchange set's ISO 8211 catalogue: eight `CATD` records
(the catalogue itself, four `.TXT` files, and `US5PSMCC.000`, `.001`, `.002`).
`test/test_s57_catalog.cpp` reads it through `S57Catalog`, the only user of the
vendored ISO 8211 reader in `src/iso8211`, and expects one dataset,
`US5PSMCC.000`, with bounds 43.05 / -70.8 / 43.125 / -70.725 (south, west,
north, east). Those values were read from the file's bytes (its subfields are
ASCII), not from ISO 8211 code. No chart cell is included: the test does not
read chart data, which goes through GDAL's own S-57 driver.

## Terms

These files are NOAA data, **not covered by this package's Apache-2.0
licence** (`marine_charts/LICENSE`). They are redistributed under NOAA's user
agreement, `enc_root/USERAGREEMENT.TXT`, which permits redistribution and asks
that the agreement be shown with redistributed copies and that NOAA's web site
be referenced (above). This copy is not an official NOAA ENC product, is not
updated, and is **not for navigation**: it is a test fixture.

`USERAGREEMENT.TXT` is kept byte for byte as NOAA ships it (mixed CRLF/LF line
endings and trailing spaces), so its sha256 can be checked against the zip. It
is therefore excluded from the `trailing-whitespace` and `mixed-line-ending`
hooks in `.pre-commit-config.yaml`, which would otherwise rewrite it.
`CATALOG.031` needs no exclusion: it holds ISO 8211 field and unit
terminators (0x1e, 0x1f), so pre-commit classes it as binary and its text
hooks skip it.
