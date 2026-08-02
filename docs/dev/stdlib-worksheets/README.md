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
| [`08-08-movement-actions.md`](08-08-movement-actions.md) | §8.8.2–§8.8.4 movement actions | 43f |

## Reading a chapter without reading all of it

Most §8 chapters spend the majority of their lines on worked examples, which are
illustrative rather than normative and are not what a translation needs. §8.8 is
2190 lines; its normative tables are 725 of them. Dropping the `Examples`
subsections first makes the difference between "too big for one sitting" and
"one sitting":

```sh
sed 's/\\//g' domain-model.md | awk 'NR>=2930 && NR<=5120' \
  | awk '/^##### .* Examples/{skip=1} /^#### /{skip=0} !skip'
```

The examples are still worth a look afterwards — §8.15.10's showed which actor
the traffic-light actions hang off, and §8.12.2's showed that `create_route` is
called with fewer arguments than it declares — but they are a second pass, not
the first one.

Chapters translated before the worksheet discipline existed (§8.7, §8.10, §8.11,
§8.12.3–§8.12.41, §8.13, §8.14) have no worksheet; their findings are recorded in
the ADR and in `frontends/dsl/README.md`.
