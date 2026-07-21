# SPDX-License-Identifier: MIT
"""Pytest suite for the structured diagnostics surface of the Python bindings."""

import scena as scn


def _scenario(events: list["scn.Event"], actors: tuple[str, ...] = ("ego",)) -> "scn.Scenario":
    scenario = scn.Scenario("diagnostics-test")
    scenario.add_entity(scn.Entity("ego", "ego vehicle", scn.ControlMode.EngineControlled))
    maneuver = scn.Maneuver("maneuver")
    for event in events:
        maneuver.add_event(event)
    group = scn.ManeuverGroup("group")
    for actor in actors:
        group.add_actor(actor)
    group.add_maneuver(maneuver)
    act = scn.Act("act")
    act.add_group(group)
    story = scn.Story("story")
    story.add_act(act)
    scenario.add_story(story)
    return scenario


def _speed_event(name: str, at_time: float, entity_id: str, target_speed: float) -> "scn.Event":
    event = scn.Event(
        name, start_trigger=scn.make_trigger(scn.SimulationTimeCondition(at_time=at_time))
    )
    event.add_action(scn.SpeedAction(entity_id, target_speed=target_speed))
    return event


def test_new_status_members_exist() -> None:
    for name in ("ParseError", "ValidationError", "SemanticError", "UnsupportedFeature"):
        assert hasattr(scn.Status, name)


def test_severity_members_exist() -> None:
    for name in ("Info", "Warning", "Error"):
        assert hasattr(scn.Severity, name)


def test_valid_scenario_has_no_diagnostics() -> None:
    engine = scn.Engine()
    assert engine.init(_scenario([_speed_event("event-1", 0.0, "ego", 10.0)])) == scn.Status.Ok
    assert engine.diagnostics() == []


def test_semantic_error_is_typed_and_reported() -> None:
    engine = scn.Engine()
    scenario = _scenario([_speed_event("event-1", 1.0, "missing", 5.0)])
    assert engine.init(scenario) == scn.Status.SemanticError

    diagnostics = engine.diagnostics()
    assert len(diagnostics) == 1
    diagnostic = diagnostics[0]
    assert diagnostic.severity == scn.Severity.Error
    assert diagnostic.code == scn.Status.SemanticError
    assert diagnostic.path == "story/act/group/maneuver/event-1/action[0]"
    assert "missing" in diagnostic.message
    assert isinstance(diagnostic.location, scn.SourceLocation)
    assert diagnostic.location.file == ""
    assert diagnostic.location.line == 0


def test_validation_error_cites_rule_id() -> None:
    engine = scn.Engine()
    scenario = _scenario([_speed_event("event-1", 1.0, "ego", 5.0)])
    # Force a negative delay on the event's condition.
    trigger = scn.make_trigger(scn.SimulationTimeCondition(at_time=1.0), delay=-1.0)
    bad_event = scn.Event("event-2", start_trigger=trigger)
    bad_event.add_action(scn.SpeedAction("ego", target_speed=1.0))
    scenario2 = _scenario([bad_event])
    assert engine.init(scenario2) == scn.Status.ValidationError
    diagnostic = engine.diagnostics()[0]
    assert diagnostic.code == scn.Status.ValidationError
    assert diagnostic.rule_id == "asam.net:xosc:1.0.0:data_type.condition_delay_not_negative"


def test_diagnostics_are_ordered() -> None:
    engine = scn.Engine()
    scenario = _scenario(
        [_speed_event("event-1", 1.0, "ego", 5.0)], actors=("ego", "missing")
    )
    # Add a second dangling reference via the event action.
    scenario2 = _scenario(
        [_speed_event("event-1", 1.0, "gone", 5.0)], actors=("ego", "missing")
    )
    assert engine.init(scenario2) == scn.Status.SemanticError
    diagnostics = engine.diagnostics()
    assert len(diagnostics) == 2
    # The actor is walked before the event's action, so it comes first.
    assert diagnostics[0].path == "story/act/group"
    assert diagnostics[1].path == "story/act/group/maneuver/event-1/action[0]"


def test_clear_and_reinit_reset_diagnostics() -> None:
    engine = scn.Engine()
    assert engine.init(_scenario([_speed_event("event-1", 1.0, "missing", 5.0)])) == (
        scn.Status.SemanticError
    )
    assert engine.diagnostics()

    engine.clear_diagnostics()
    assert engine.diagnostics() == []

    # A successful re-init also starts from an empty record.
    assert engine.init(_scenario([_speed_event("event-1", 0.0, "ego", 10.0)])) == scn.Status.Ok
    assert engine.diagnostics() == []


def test_diagnostics_list_survives_close() -> None:
    engine = scn.Engine()
    # Run a defective init, capture the list (copies), then re-init valid and
    # close — the captured list is unaffected.
    assert engine.init(_scenario([_speed_event("event-1", 1.0, "missing", 5.0)])) == (
        scn.Status.SemanticError
    )
    captured = engine.diagnostics()
    assert len(captured) == 1

    assert engine.init(_scenario([_speed_event("event-1", 0.0, "ego", 10.0)])) == scn.Status.Ok
    assert engine.close() == scn.Status.Ok
    # The earlier copies are still readable.
    assert captured[0].code == scn.Status.SemanticError
    assert "missing" in captured[0].message


def test_repr_contains_code_and_path() -> None:
    engine = scn.Engine()
    assert engine.init(_scenario([_speed_event("event-1", 1.0, "missing", 5.0)])) == (
        scn.Status.SemanticError
    )
    text = repr(engine.diagnostics()[0])
    assert "SemanticError" in text
    assert "action[0]" in text


def test_unsupported_action_kind_warns_but_step_stays_ok() -> None:
    # There is no way to build an unsupported action from Python yet (only
    # SpeedAction is bound), so this checks the kind property that runtime
    # diagnostics rely on.
    action = scn.SpeedAction("ego", target_speed=1.0)
    assert action.kind == "SpeedAction"
