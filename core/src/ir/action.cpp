// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#include "scena/ir/action.h"

#include <utility>

namespace scena::ir {

SpeedAction::SpeedAction(std::string entity_id, double target_speed)
    : entity_id_(std::move(entity_id)), target_speed_(target_speed) {}
// dynamics_ defaults to a Step transition, so this reaches the target
// instantaneously — the historical SpeedAction behaviour.

SpeedAction::SpeedAction(std::string entity_id, double target_speed, TransitionDynamics dynamics)
    : entity_id_(std::move(entity_id)), target_speed_(target_speed), dynamics_(dynamics) {}

SpeedAction::SpeedAction(std::string entity_id, RelativeTargetSpeed target,
                         TransitionDynamics dynamics)
    : entity_id_(std::move(entity_id)), target_speed_(0.0), relative_target_(std::move(target)),
      dynamics_(dynamics) {}

const std::string& SpeedAction::entity_id() const {
    return entity_id_;
}

std::string_view SpeedAction::kind() const noexcept {
    return "SpeedAction";
}

bool SpeedAction::is_relative() const noexcept {
    return relative_target_.has_value();
}

double SpeedAction::target_speed() const {
    return target_speed_;
}

const std::optional<RelativeTargetSpeed>& SpeedAction::relative_target() const {
    return relative_target_;
}

const TransitionDynamics& SpeedAction::dynamics() const {
    return dynamics_;
}

SpeedProfileAction::SpeedProfileAction(std::string entity_id,
                                       std::vector<SpeedProfileEntry> entries,
                                       FollowingMode following_mode)
    : entity_id_(std::move(entity_id)), entries_(std::move(entries)),
      following_mode_(following_mode) {}

const std::string& SpeedProfileAction::entity_id() const {
    return entity_id_;
}

std::string_view SpeedProfileAction::kind() const noexcept {
    return "SpeedProfileAction";
}

const std::vector<SpeedProfileEntry>& SpeedProfileAction::entries() const {
    return entries_;
}

FollowingMode SpeedProfileAction::following_mode() const {
    return following_mode_;
}

TeleportAction::TeleportAction(std::string entity_id, WorldPosition position)
    : entity_id_(std::move(entity_id)), position_(position) {}

const std::string& TeleportAction::entity_id() const {
    return entity_id_;
}

std::string_view TeleportAction::kind() const noexcept {
    return "TeleportAction";
}

const WorldPosition& TeleportAction::position() const {
    return position_;
}

LongitudinalDistanceAction::LongitudinalDistanceAction(
    std::string entity_id, std::string entity_ref, std::optional<double> distance,
    std::optional<double> time_gap, bool freespace, bool continuous,
    CoordinateSystem coordinate_system, LongitudinalDisplacement displacement,
    std::optional<DynamicConstraints> constraints)
    : entity_id_(std::move(entity_id)), entity_ref_(std::move(entity_ref)), distance_(distance),
      time_gap_(time_gap), freespace_(freespace), continuous_(continuous),
      coordinate_system_(coordinate_system), displacement_(displacement),
      constraints_(std::move(constraints)) {}

const std::string& LongitudinalDistanceAction::entity_id() const {
    return entity_id_;
}

std::string_view LongitudinalDistanceAction::kind() const noexcept {
    return "LongitudinalDistanceAction";
}

const std::string& LongitudinalDistanceAction::entity_ref() const {
    return entity_ref_;
}

const std::optional<double>& LongitudinalDistanceAction::distance() const {
    return distance_;
}

const std::optional<double>& LongitudinalDistanceAction::time_gap() const {
    return time_gap_;
}

bool LongitudinalDistanceAction::freespace() const noexcept {
    return freespace_;
}

bool LongitudinalDistanceAction::continuous() const noexcept {
    return continuous_;
}

CoordinateSystem LongitudinalDistanceAction::coordinate_system() const noexcept {
    return coordinate_system_;
}

LongitudinalDisplacement LongitudinalDistanceAction::displacement() const noexcept {
    return displacement_;
}

const std::optional<DynamicConstraints>& LongitudinalDistanceAction::constraints() const {
    return constraints_;
}

AssignRouteAction::AssignRouteAction(std::string entity_id, Route route)
    : entity_id_(std::move(entity_id)), route_(std::move(route)) {}

const std::string& AssignRouteAction::entity_id() const {
    return entity_id_;
}

std::string_view AssignRouteAction::kind() const noexcept {
    return "AssignRouteAction";
}

const Route& AssignRouteAction::route() const {
    return route_;
}

AcquirePositionAction::AcquirePositionAction(std::string entity_id, WorldPosition position)
    : entity_id_(std::move(entity_id)), position_(position) {}

const std::string& AcquirePositionAction::entity_id() const {
    return entity_id_;
}

std::string_view AcquirePositionAction::kind() const noexcept {
    return "AcquirePositionAction";
}

const WorldPosition& AcquirePositionAction::position() const {
    return position_;
}

FollowTrajectoryAction::FollowTrajectoryAction(std::string entity_id, Trajectory trajectory,
                                               FollowingMode following_mode,
                                               std::optional<Timing> time_reference,
                                               double initial_distance_offset)
    : entity_id_(std::move(entity_id)), trajectory_(std::move(trajectory)),
      following_mode_(following_mode), time_reference_(time_reference),
      initial_distance_offset_(initial_distance_offset) {}

const std::string& FollowTrajectoryAction::entity_id() const {
    return entity_id_;
}

std::string_view FollowTrajectoryAction::kind() const noexcept {
    return "FollowTrajectoryAction";
}

const Trajectory& FollowTrajectoryAction::trajectory() const {
    return trajectory_;
}

FollowingMode FollowTrajectoryAction::following_mode() const noexcept {
    return following_mode_;
}

const std::optional<Timing>& FollowTrajectoryAction::time_reference() const {
    return time_reference_;
}

double FollowTrajectoryAction::initial_distance_offset() const noexcept {
    return initial_distance_offset_;
}

AssignControllerAction::AssignControllerAction(std::string entity_id, Controller controller,
                                               std::optional<bool> activate_lateral,
                                               std::optional<bool> activate_longitudinal)
    : entity_id_(std::move(entity_id)), controller_(std::move(controller)),
      activate_lateral_(activate_lateral), activate_longitudinal_(activate_longitudinal) {}

const std::string& AssignControllerAction::entity_id() const {
    return entity_id_;
}

std::string_view AssignControllerAction::kind() const noexcept {
    return "AssignControllerAction";
}

const Controller& AssignControllerAction::controller() const {
    return controller_;
}

const std::optional<bool>& AssignControllerAction::activate_lateral() const {
    return activate_lateral_;
}

const std::optional<bool>& AssignControllerAction::activate_longitudinal() const {
    return activate_longitudinal_;
}

ActivateControllerAction::ActivateControllerAction(std::string entity_id,
                                                   std::optional<bool> lateral,
                                                   std::optional<bool> longitudinal)
    : entity_id_(std::move(entity_id)), lateral_(lateral), longitudinal_(longitudinal) {}

const std::string& ActivateControllerAction::entity_id() const {
    return entity_id_;
}

std::string_view ActivateControllerAction::kind() const noexcept {
    return "ActivateControllerAction";
}

const std::optional<bool>& ActivateControllerAction::lateral() const {
    return lateral_;
}

const std::optional<bool>& ActivateControllerAction::longitudinal() const {
    return longitudinal_;
}

VisibilityAction::VisibilityAction(std::string entity_id, bool graphics, bool sensors, bool traffic)
    : entity_id_(std::move(entity_id)), graphics_(graphics), sensors_(sensors), traffic_(traffic) {}

const std::string& VisibilityAction::entity_id() const {
    return entity_id_;
}

std::string_view VisibilityAction::kind() const noexcept {
    return "VisibilityAction";
}

bool VisibilityAction::graphics() const noexcept {
    return graphics_;
}

bool VisibilityAction::sensors() const noexcept {
    return sensors_;
}

bool VisibilityAction::traffic() const noexcept {
    return traffic_;
}

} // namespace scena::ir
