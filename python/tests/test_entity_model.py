# SPDX-License-Identifier: MIT
"""Entity taxonomy (ASAM OpenSCENARIO XML 1.4.0 §7.2.2) through the bindings."""

import scena as scn


def make_vehicle():
    return scn.Vehicle(
        category=scn.VehicleCategory.Car,
        role=scn.Role.Civil,
        mass=1500.0,
        bounding_box=scn.BoundingBox(center_x=1.4, center_z=0.8, length=4.6, width=2.0, height=1.5),
        performance=scn.Performance(max_speed=60.0, max_acceleration=5.0, max_deceleration=9.0),
        axles=scn.Axles(rear=scn.Axle(position_x=2.8, track_width=1.6, wheel_diameter=0.6)),
        properties=[scn.Property("color", "blue")],
    )


def test_vehicle_round_trips_and_derives_metadata():
    entity = scn.Entity("ego", "ego", scn.ControlMode.HostControlled, object=make_vehicle())

    assert entity.object_type == scn.ObjectType.Vehicle
    assert isinstance(entity.object, scn.Vehicle)
    assert entity.object.category == scn.VehicleCategory.Car
    assert entity.object.role == scn.Role.Civil
    assert entity.object.mass == 1500.0
    assert entity.object.properties[0].value == "blue"
    assert entity.object.axles.rear.position_x == 2.8
    assert entity.object.axles.front is None

    # Derived read-only views.
    assert entity.bounding_box.length == 4.6
    assert entity.performance.max_speed == 60.0
    # An absent optional rate reads back as None.
    assert entity.performance.max_acceleration_rate is None


def test_pedestrian_and_misc_object_types():
    ped = scn.Entity("ped", "ped", object=scn.Pedestrian(category=scn.PedestrianCategory.Pedestrian))
    misc = scn.Entity("m", "m", object=scn.MiscObject(category=scn.MiscObjectCategory.Pole))

    assert ped.object_type == scn.ObjectType.Pedestrian
    assert misc.object_type == scn.ObjectType.MiscObject
    # Only a vehicle exposes performance.
    assert ped.performance is None
    assert misc.performance is None


def test_bare_participant_has_no_object():
    bare = scn.Entity("ghost", "ghost", scn.ControlMode.HostControlled)
    assert bare.object is None
    assert bare.object_type is None
    assert bare.bounding_box is None
    assert bare.performance is None


def test_none_role_and_category_exposed_as_NONE():
    # Python forbids `None` as an attribute name, so the spec's "none" is NONE.
    assert scn.Role.NONE is not None
    assert scn.MiscObjectCategory.NONE is not None
    assert scn.Vehicle().role == scn.Role.NONE


def test_full_pose_round_trips_through_engine():
    scenario = scn.Scenario("pose")
    scenario.add_entity(scn.Entity("ego", "ego", scn.ControlMode.HostControlled, object=make_vehicle()))
    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok

    engine.report_state("ego", scn.EntityState(x=1.0, heading=0.3, speed=2.0, pitch=0.1, roll=-0.2))
    assert engine.step(0.02) == scn.Status.Ok
    state = engine.state("ego")
    assert state is not None
    assert state.pitch == 0.1
    assert state.roll == -0.2


def test_invalid_performance_is_a_validation_error():
    scenario = scn.Scenario("bad")
    bad = scn.Vehicle(performance=scn.Performance(max_speed=-1.0, max_acceleration=1.0, max_deceleration=1.0))
    scenario.add_entity(scn.Entity("v", "v", scn.ControlMode.HostControlled, object=bad))
    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.ValidationError
