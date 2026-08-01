# ADR-0023: Catalogs, entity selections and controller assignment (p4-s4)

## Status

Accepted (p4-s4, #27).

## Context

Catalogs (§9.4–9.6) let a scenario outsource elements to separate files;
entity selections (§7.2.2.2) let it name a set of entities once; object
controllers (§6.6) let it say who drives an entity. All three are *reuse*
mechanisms — they change where a definition lives, not what it means — and
all three touch things Scena is strict about: the filesystem (whose
enumeration order is unspecified), parameter scoping, and control ownership.

## Decision

- **A catalog directory is scanned once, in sorted file-name order.**
  `std::filesystem::directory_iterator` yields entries in an unspecified
  order that differs between filesystems, so the scan collects the file
  names, sorts them, and only then parses. A duplicate `(catalog, entry)`
  name resolves to the first in that order, with a warning naming it.
  Without the sort, which of two same-named entries a scenario got would
  depend on the machine it ran on — a determinism break before the engine
  ever starts.
- **Parsed catalog documents outlive the reference.** The cache owns them,
  so a resolved entry is a node in a live document and the element readers
  run on it exactly as they would on an inline element. One reader per
  element, whether the element came from the scenario or from a catalog.
- **A catalog entry's parameter frame is isolated.** §9.5 is explicit: "no
  other parameters may be referenced from within the catalog". `ParameterScope`
  gained isolated frames, and lookup stops at that boundary — an entry
  cannot silently pick up a scenario parameter that shares a name.
  Assignment *values*, though, are read in the referencing scope before the
  isolation begins, because the standard notes an assignment value may
  itself be a parameter reference.
- **The entry's declarations are applied once.** `CatalogEntryScope` reads
  them as defaults and overrides them with the assignments; the element
  reader that runs next would otherwise read the same `ParameterDeclarations`
  into its own nested frame and shadow the assignments with the defaults, so
  the scope marks them applied and the declaration reader consumes that
  mark.
- **A ScenarioObject's catalog reference is resolved by trying the object
  kinds in turn.** The reference names a catalog and an entry, not which of
  Vehicle/Pedestrian/MiscObject it is, so the loader tries the three
  directories in the order the `ScenarioObject` choice declares them. A miss
  against one kind is silent — it is how the right one is found — and only
  "no kind holds it" is a diagnostic.
- **Entity selections expand at load time.** The Scenario IR names entities
  individually: a selection is a way of *writing* a set, not a runtime
  object. Expansion happens in declaration order into ManeuverGroup actors
  (where §8.3.3.3 then applies the private actions per member) and into a
  condition's triggering entities (where any/all reduces over the members).
  Selections are read after every `ScenarioObject`, so members are always
  known, and a selection naming another one sees only those declared before
  it — which also breaks the circular definitions §7.2.2.2 warns about.
- **An ObjectController is metadata for the host, not a change of control
  ownership.** It lowers to an `AssignControllerAction` at init, so the
  controller's name, type and properties reach the gateway through the same
  path a storyboard `AssignControllerAction` uses. The entity stays
  engine-controlled: a scenario says *what* should drive an entity, and only
  the embedder knows whether that thing exists in this simulation
  (ADR-0003). Making a scenario file able to hand an entity to a host that
  may not be there would produce an entity nothing drives.

## Consequences

- The committed fixture tree under `frontends/xml/tests/catalogs/` loads
  reproducibly — the sprint's exit criterion — and the test asserts equality
  between two loads rather than trusting the sort by inspection.
- p4-s5's file-level validation has the reference machinery it needs:
  catalog references either resolve or are reported with their rule id, so
  the referential-integrity pass can assume resolved content.
- A selection used as a *single-entity* `entityRef` on an action is not
  expanded (the IR action carries one entity id); actors and triggering
  entities are where the standard's bulk semantics live, and those are
  covered. The limit is documented in the user guide rather than silently
  applied.
- `ParameterScope` grew isolated frames and `ReadContext` a
  declarations-applied mark; both are internal to the frontend and cost
  nothing outside a catalog entry.
