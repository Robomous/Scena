# Catalogs, entity selections and controllers

Three mechanisms for writing scenarios that reuse things: **catalogs**
(§9.4–9.6) outsource an element to a separate file, **entity selections**
(§7.2.2.2) name a set of entities once, and **object controllers** (§6.6)
declare who drives an entity.

## Catalogs

A catalog file holds reusable elements under a named `Catalog`:

```xml
<OpenSCENARIO>
  <FileHeader revMajor="1" revMinor="2" .../>
  <Catalog name="ScenaVehicles">
    <Vehicle name="compact" vehicleCategory="car"> ... </Vehicle>
  </Catalog>
</OpenSCENARIO>
```

A scenario declares one directory per catalog kind and then references
entries by catalog and entry name:

```xml
<CatalogLocations>
  <VehicleCatalog><Directory path="catalogs/vehicles"/></VehicleCatalog>
</CatalogLocations>
...
<ScenarioObject name="ego">
  <CatalogReference catalogName="ScenaVehicles" entryName="compact"/>
</ScenarioObject>
```

Scena resolves references for the eight catalog kinds it executes: vehicle,
controller, pedestrian, miscObject, environment, maneuver, trajectory and
route. (`TrafficDistributionEntry` belongs to the traffic family, which is
Post-v0.0.1.)

**Directory paths are relative to the scenario file.** A document loaded
from memory (`load_string`) has no location on disk, so a relative catalog
directory cannot be resolved and the reference is reported rather than
guessed.

### Reproducibility

A directory is scanned once, the first time a reference needs it. Because
`std::filesystem` enumerates a directory in an unspecified order that
differs between filesystems, Scena **sorts the file names before parsing
anything**. Two loads of the same tree therefore see the same catalogs in
the same order, and a duplicate `(catalog, entry)` name always resolves to
the same entry — the first in file-name order, with a warning that names the
duplicate. The IR never depends on how a filesystem happened to enumerate.

Failures cite their rule: an unresolvable entry
`reference_control.catalog_reference_resolvability`, a missing or unreadable
directory `reference_control.catalogs_referenced_by_directory`.

### Parameters in catalogs

A catalog entry declares its own parameters with default values, and a
reference overrides them (§9.5):

```xml
<Vehicle name="compact" ...>
  <ParameterDeclarations>
    <ParameterDeclaration name="maxSpeed" parameterType="double" value="50.0"/>
  </ParameterDeclarations>
  <Performance maxSpeed="$maxSpeed" .../>
</Vehicle>
```

```xml
<CatalogReference catalogName="ScenaVehicles" entryName="compact">
  <ParameterAssignments>
    <ParameterAssignment parameterRef="maxSpeed" value="70.0"/>
  </ParameterAssignments>
</CatalogReference>
```

Two rules matter here:

- **A catalog entry sees only its own parameters.** "No other parameters may
  be referenced from within the catalog" — a scenario parameter that happens
  to share a name is deliberately invisible inside the entry, so an entry
  cannot pick up a value its author never declared.
- **An assignment's value is evaluated in the referencing scenario**, before
  that isolation begins, so `value="$fleet_speed"` means the scenario's
  `fleet_speed`. The standard notes this explicitly.

## Entity selections

An `EntitySelection` names a set of entities:

```xml
<EntitySelection name="all_vehicles">
  <Members><ByType objectType="vehicle"/></Members>
</EntitySelection>
```

Members are named explicitly with `EntityRef`, or matched by object type
with `ByType`; a selection may also name another selection. Scena expands a
selection **at load time**, in declaration order, and a member named twice
inside one selection counts once.

A selection may be used where a single entity may:

- as a **ManeuverGroup actor**, where the group's private actions then apply
  individually to each member (§7.2.2.2, §8.3.3.3);
- as a **triggering entity**, where the `any`/`all` reduction runs over the
  members.

Because the expansion happens at load time and selections are read after the
`ScenarioObject`s they name, a selection that names another one sees only
the selections declared before it — which is also what breaks the circular
definitions §7.2.2.2 warns about.

## Object controllers

A `ScenarioObject` may declare an `ObjectController`, inline or from the
controller catalog:

```xml
<ScenarioObject name="ego">
  <Vehicle .../>
  <ObjectController>
    <CatalogReference catalogName="ScenaControllers" entryName="external_driver"/>
  </ObjectController>
</ScenarioObject>
```

Scena lowers that to an `AssignControllerAction` applied at init: the
controller's name, type and properties reach the host through
`ISimulatorGateway::on_controller_assigned`, exactly as an
`AssignControllerAction` in the storyboard would.

**Control ownership does not change.** The entity stays engine-controlled;
whether the host actually drives it is the embedder's decision, made through
the [gateway](../architecture/ADR-0003-simulator-gateway.md), not the
scenario's. A controller in a scenario file says *what* should drive an
entity, and only the host knows whether that thing exists in this
simulation. See [control ownership](positions.md) for how an entity becomes
host-controlled.

## See also

- [Loading scenarios](loading-scenarios.md) — the loader and its
  diagnostics.
- [Parameters, expressions and variables](parameters.md) — the scoping
  rules catalogs build on.
- [ADR-0023](../architecture/ADR-0023-catalogs-and-controllers.md) — the
  decisions behind this chapter.
