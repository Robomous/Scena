// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <optional>
#include <string>

#include "scena/ir/date_time.h"

namespace scena::ir {

/// The simulated day and time, per ASAM OpenSCENARIO XML 1.4.0 §TimeOfDay.
///
/// `animation` decides whether the clock runs: "If true, the timeofday is
/// animated with progressing simulation time, e.g. in order to animate the
/// position of the sun." With it false the simulated instant is frozen at
/// `date_time`, which a TimeOfDayCondition then compares against forever.
struct TimeOfDay {
    bool animation = false;
    DateTime date_time;
};

/// Kinds of precipitation, per §PrecipitationType.
enum class PrecipitationType {
    Dry,  ///< No precipitation.
    Rain, ///< Rain.
    Snow, ///< Snow.
};

/// Sun position and intensity, per §Sun. Angles are radians in the world
/// coordinate system; `illuminance` defaults to the §Sun default of 0.
struct Sun {
    /// Counted clockwise, 0 = north, PI/2 = east. Unit: [rad]. Range: [0..2*PI].
    double azimuth = 0.0;
    /// 0 = x/y plane, PI/2 = zenith. Unit: [rad]. Range: [-PI..PI].
    double elevation = 0.0;
    /// Direct sunlight is around 100 000 lx. Unit: [lx]. Range: [0..inf[.
    double illuminance = 0.0;
};

/// Fog, per §Fog. The optional fog bounding box is not modeled — Scena stores
/// environment state and couples it to no physics, so a fog volume would carry
/// no meaning here.
struct Fog {
    /// Unit: [m]. Range: [0..inf[.
    double visual_range = 0.0;
};

/// Precipitation type and intensity, per §Precipitation.
struct Precipitation {
    PrecipitationType type = PrecipitationType::Dry;
    /// Valid for all precipitation types. Unit: [mm/h]. Range: [0..inf[.
    double intensity = 0.0;
};

/// Wind direction and speed, per §Wind.
struct Wind {
    /// Target direction (not the origin direction) in the ground plane;
    /// x-axis is 0. Unit: [rad]. Range: [0..2*PI[.
    double direction = 0.0;
    /// Unit: [m/s]. Range: [0..inf[.
    double speed = 0.0;
};

/// Weather state, per §Weather. Every member is optional, and the standard is
/// explicit about what an absent one means: "If one of the conditions is
/// missing it means that it doesn't change." An EnvironmentAction therefore
/// merges member by member rather than replacing the whole Weather.
///
/// The §DomeImage sky reference is not modeled (an image is a rendering
/// concern, and it is mutually exclusive with the members that are).
struct Weather {
    std::optional<Sun> sun;
    std::optional<Fog> fog;
    std::optional<Precipitation> precipitation;
    std::optional<Wind> wind;
    /// Outside temperature at z = 0. Unit: [K]. Range: [170..340].
    std::optional<double> temperature;
    /// Reference atmospheric pressure at z = 0. Unit: [Pa]. Range:
    /// [80000..120000].
    std::optional<double> atmospheric_pressure;
    /// Cloud cover in oktas, 0 (clear) to 9 (sky obscured) [1.2]. The standard
    /// spells this as the §FractionalCloudCover enumeration
    /// (zeroOktas..nineOktas); the IR carries the okta count itself, which maps
    /// onto those literals one for one.
    std::optional<int> fractional_cloud_cover_oktas;
};

/// Road surface condition, per §RoadCondition. Only the required friction
/// scale factor is modeled: §Wetness [1.2] and the free-form §Properties
/// describe surfaces Scena's point-mass motion model has no use for, and
/// storing them without acting on them would overstate coverage.
struct RoadCondition {
    /// Range: [0..inf[; 1.0 is nominal friction.
    double friction_scale_factor = 1.0;
};

/// Environment state, per §Environment: "Defines the environment conditions of
/// a scenario, e.g. time of day, weather and road condition. If one of the
/// conditions is missing it [does not change]."
///
/// Scena stores this and couples it to no physics or rendering: the engine has
/// neither. The one member with runtime meaning is `time_of_day`, which feeds
/// the simulated clock a TimeOfDayCondition reads. Everything else is state a
/// host reads back through Engine::environment and acts on itself. The
/// §Environment parameterDeclarations belong to catalog instantiation (P4).
struct Environment {
    std::string name;
    std::optional<TimeOfDay> time_of_day;
    std::optional<Weather> weather;
    std::optional<RoadCondition> road_condition;
};

} // namespace scena::ir
