# SPDX-License-Identifier: Apache-2.0
"""Entity taxonomy example: build classified entities and query their metadata.

Mirrors the C ABI builders and read-back accessors (ASAM OpenSCENARIO XML
1.4.0 §7.2.2). Runnable and self-asserting; doubles as a smoke test.
"""

import scena as scn


def main() -> None:
    scenario = scn.Scenario("entity-model")

    # A car with a bounding box, performance limits, and one axle set.
    ego = scn.Vehicle(
        category=scn.VehicleCategory.Car,
        role=scn.Role.NONE,
        bounding_box=scn.BoundingBox(center_x=1.4, center_z=0.8, length=4.6, width=2.0, height=1.5),
        performance=scn.Performance(
            max_speed=55.0, max_acceleration=4.0, max_deceleration=9.0, max_acceleration_rate=3.0
        ),
        axles=scn.Axles(rear=scn.Axle(position_x=2.7, track_width=1.6, wheel_diameter=0.6)),
        properties=[scn.Property("color", "silver")],
    )
    ego_entity = scn.Entity("ego", "ego", scn.ControlMode.HostControlled, object=ego)
    scenario.add_entity(ego_entity)

    # A pedestrian and a static pole — neither has performance.
    walker = scn.Entity(
        "walker",
        "walker",
        scn.ControlMode.HostControlled,
        object=scn.Pedestrian(
            category=scn.PedestrianCategory.Pedestrian,
            bounding_box=scn.BoundingBox(center_z=0.9, length=0.5, width=0.6, height=1.8),
        ),
    )
    scenario.add_entity(walker)
    pole = scn.Entity(
        "pole",
        "pole",
        scn.ControlMode.HostControlled,
        object=scn.MiscObject(
            category=scn.MiscObjectCategory.Pole,
            bounding_box=scn.BoundingBox(center_z=1.5, length=0.2, width=0.2, height=3.0),
        ),
    )
    scenario.add_entity(pole)

    # Metadata is derivable straight off each entity.
    assert ego_entity.object_type == scn.ObjectType.Vehicle
    assert ego_entity.performance.max_speed == 55.0
    assert walker.object_type == scn.ObjectType.Pedestrian
    assert walker.performance is None
    assert pole.bounding_box.height == 3.0

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    # The host reports a full pose; pitch/roll survive the round-trip.
    engine.report_state("ego", scn.EntityState(x=2.0, speed=10.0, pitch=0.05, roll=-0.02))
    assert engine.step(0.1) == scn.Status.Ok
    state = engine.state("ego")
    assert state is not None and state.pitch == 0.05

    print("entity model: vehicle/pedestrian/misc built, metadata queried, pose round-tripped")


if __name__ == "__main__":
    main()
