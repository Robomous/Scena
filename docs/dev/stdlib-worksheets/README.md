# Standard-library translation worksheets

Working notes for the bundled ASAM OpenSCENARIO DSL §8 standard library
(`frontends/dsl/src/stdlib.cpp`, ADR-0029).

Most of §8 is printed as *parameter tables*, not as DSL code. Turning a chapter
into library source is therefore a translation, one table row at a time, and the
expensive part is reading the chapter rather than writing the declarations. A
worksheet captures that reading once: one line per declaration, in the order the
chapter prints them, with the resulting DSL spelling and a `⚠` note wherever the
printed text needs a judgement call.

The worksheets are committed for two reasons:

- they are the auditable record of the translation — a reviewer can check a
  worksheet row against the specification without re-deriving the mapping from
  the library source; and
- they are the recovery path. Re-reading a chapter costs far more than reading
  its worksheet.

They are developer notes, not user documentation: `docs/user-guide/` describes
what the frontend does, and `docs/roadmap/coverage/osc-dsl-coverage.md` records
what of the standard is covered. A worksheet only records how one chapter was
translated.

| Worksheet | Chapter | Landed in |
| --- | --- | --- |
| [`08-12-02-map.md`](08-12-02-map.md) | §8.12.2 actor `map` | 43e |
| [`08-15-traffic-lights.md`](08-15-traffic-lights.md) | §8.15 traffic lights | 43h |

Chapters translated before the worksheet discipline existed (§8.7, §8.10, §8.11,
§8.12.3–§8.12.41, §8.13, §8.14) have no worksheet; their findings are recorded in
the ADR and in `frontends/dsl/README.md`.
