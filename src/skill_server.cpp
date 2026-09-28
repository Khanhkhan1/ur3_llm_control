#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ur3_llm_control/srv/home.hpp"
#include "ur3_llm_control/srv/pick.hpp"
#include "ur3_llm_control/srv/place.hpp"

using moveit::planning_interface::MoveGroupInterface;

namespace {

constexpr int kMaxAttempts = 15;

geometry_msgs::msg::Quaternion downOrientation()
{
  tf2::Quaternion q;
  q.setRPY(M_PI, 0, 0);
  return tf2::toMsg(q);
}

geometry_msgs::msg::Pose poseAt(double x, double y, double z)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation = downOrientation();
  return p;
}

// Shift wrist_3_joint (continuous, no position limits) so consecutive
// trajectory points never jump by more than pi -- avoids the controller
// spinning the wrist all the way around between numerically-independent
// IK solutions that are the same physical orientation.
// Unwraps each point relative to the previous one, self-seeded from the
// trajectory's own first point (no external "current state" needed).
void unwrapContinuousJoint(trajectory_msgs::msg::JointTrajectory & traj, const std::string & joint_name)
{
  auto it = std::find(traj.joint_names.begin(), traj.joint_names.end(), joint_name);
  if (it == traj.joint_names.end()) return;
  size_t idx = std::distance(traj.joint_names.begin(), it);
  if (traj.points.empty()) return;

  double prev = traj.points.front().positions[idx];
  for (auto & point : traj.points) {
    double & v = point.positions[idx];
    while (v - prev > M_PI) v -= 2 * M_PI;
    while (v - prev < -M_PI) v += 2 * M_PI;
    prev = v;
  }
}

bool withRetries(
  const std::function<bool()> & attempt, const rclcpp::Logger & logger, const std::string & label)
{
  for (int i = 1; i <= kMaxAttempts; ++i) {
    if (attempt()) return true;
    RCLCPP_WARN(logger, "%s: attempt %d/%d failed, retrying...", label.c_str(), i, kMaxAttempts);
  }
  RCLCPP_ERROR(logger, "%s: giving up after %d attempts", label.c_str(), kMaxAttempts);
  return false;
}

}  // namespace

class SkillServerNode : public rclcpp::Node
{
public:
  SkillServerNode() : rclcpp::Node("skill_server")
  {
    declare_parameter("planning_group", "ur_manipulator");
    declare_parameter("approach_height", 0.12);
    declare_parameter("pick_descend_z", 0.22);
    declare_parameter("place_descend_z", 0.22);
    declare_parameter("velocity_scaling", 0.3);
    declare_parameter("acceleration_scaling", 0.3);
    declare_parameter<std::vector<std::string>>("objects.names", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("zones.names", std::vector<std::string>{});

    // MoveGroupInterface::getCurrentState()/getCurrentPose() go through a
    // "current state monitor" that rejects a /joint_states message unless
    // its header.stamp is >= the time it started waiting -- on this setup
    // joint_state_broadcaster was observed to publish header.stamp as 0
    // while /clock (sim time) is already far ahead, so that check never
    // passes and those calls fail with "Failed to fetch current robot
    // state" every time. Subscribing directly here and building our own
    // RobotState from the latest message (no freshness requirement)
    // sidesteps that entirely -- see cartesianTo().
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) { latest_joint_state_ = msg; });
  }

  // Parameter overrides from a `/**:`-scoped --params-file are not
  // guaranteed to be visible yet at construction time (observed empirically
  // on this setup: get_parameter() in the constructor still saw the
  // declared defaults, even though the override was present moments
  // later). Reading them here -- after the node has been added to the
  // executor and spinning has started -- reliably sees the real values.
  void initMoveGroup()
  {
    // NOTE: get_parameter() here was observed (empirically, on this
    // ROS 2 Humble / RoboStack build) to still return declare-time
    // defaults immediately after declare_parameter(), even though the
    // --params-file override for that same name is already correct
    // moments later (e.g. via `ros2 param get`). Reading directly from
    // the raw override map via get_parameter_overrides() sidesteps
    // whatever that timing gap is, and is available as soon as the node
    // is constructed.
    auto overrides = get_node_parameters_interface()->get_parameter_overrides();
    auto ov_double = [&overrides](const std::string & name, double fallback) {
      auto it = overrides.find(name);
      return it != overrides.end() ? it->second.get<double>() : fallback;
    };
    auto ov_string_array = [&overrides](const std::string & name) {
      auto it = overrides.find(name);
      return it != overrides.end() ? it->second.get<std::vector<std::string>>()
                                    : std::vector<std::string>{};
    };

    approach_height_ = ov_double("approach_height", approach_height_default_);
    pick_descend_z_ = ov_double("pick_descend_z", pick_descend_z_default_);
    place_descend_z_ = ov_double("place_descend_z", place_descend_z_default_);
    vel_scale_ = ov_double("velocity_scaling", vel_scale_default_);
    accel_scale_ = ov_double("acceleration_scaling", accel_scale_default_);
    {
      auto it = overrides.find("planning_group");
      planning_group_ = it != overrides.end() ? it->second.get<std::string>() : planning_group_;
    }

    for (const auto & name : ov_string_array("objects.names")) {
      double x = ov_double("objects." + name + ".x", 0.0);
      double y = ov_double("objects." + name + ".y", 0.0);
      declare_parameter("objects." + name + ".x", x);
      declare_parameter("objects." + name + ".y", y);
      objects_[name] = {x, y};
    }
    for (const auto & name : ov_string_array("zones.names")) {
      double x = ov_double("zones." + name + ".x", 0.0);
      double y = ov_double("zones." + name + ".y", 0.0);
      declare_parameter("zones." + name + ".x", x);
      declare_parameter("zones." + name + ".y", y);
      zones_[name] = {x, y};
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu objects, %zu zones", objects_.size(), zones_.size());

    move_group_ = std::make_shared<MoveGroupInterface>(shared_from_this(), planning_group_);
    move_group_->setMaxVelocityScalingFactor(vel_scale_);
    move_group_->setMaxAccelerationScalingFactor(accel_scale_);
    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(10);

    addTableCollisionObject();
    publishSceneMarkers();

    home_srv_ = create_service<ur3_llm_control::srv::Home>(
      "skill/home",
      std::bind(&SkillServerNode::handleHome, this, std::placeholders::_1, std::placeholders::_2));
    pick_srv_ = create_service<ur3_llm_control::srv::Pick>(
      "skill/pick",
      std::bind(&SkillServerNode::handlePick, this, std::placeholders::_1, std::placeholders::_2));
    place_srv_ = create_service<ur3_llm_control::srv::Place>(
      "skill/place",
      std::bind(&SkillServerNode::handlePlace, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "skill_server ready: /skill/home, /skill/pick, /skill/place");
  }

private:
  // See the joint_state_sub_ subscription (constructor) for why this
  // exists instead of move_group_->getCurrentState(). Falls back to
  // whatever the robot model's default values are if no /joint_states
  // has arrived yet (shouldn't happen in practice -- MoveGroupInterface
  // itself won't be ready that early either).
  moveit::core::RobotStatePtr buildKnownState()
  {
    auto state = std::make_shared<moveit::core::RobotState>(move_group_->getRobotModel());
    state->setToDefaultValues();
    if (latest_joint_state_) {
      state->setVariableValues(*latest_joint_state_);
    }
    state->update();
    return state;
  }

  // The table (from worlds/tabletop.sdf) is a real physical obstacle in
  // Gazebo, but MoveIt's planning scene has no knowledge of anything
  // outside the robot's own URDF unless told about it explicitly -- an
  // OMPL-planned joint-space move (e.g. from the "up" pose to a pose
  // hovering over the table) can therefore be collision-free from
  // MoveIt's point of view while physically driving the arm straight
  // into the table in the simulator. Publishing the table's geometry
  // here as a static CollisionObject makes OMPL route around it.
  // Deliberately not adding the cubes themselves: pick/place needs to
  // descend to exactly their location, which collision checking would
  // then always reject.
  void addTableCollisionObject()
  {
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = move_group_->getPlanningFrame();
    table.id = "table";

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {0.5, 0.8, 0.20};

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.35;
    pose.position.y = 0.0;
    pose.position.z = 0.10;
    pose.orientation.w = 1.0;

    table.primitives.push_back(box);
    table.primitive_poses.push_back(pose);
    table.operation = moveit_msgs::msg::CollisionObject::ADD;

    planning_scene_interface_.applyCollisionObject(table);
    RCLCPP_INFO(get_logger(), "Added 'table' collision object to the planning scene");
  }

  // Purely visual: the cubes/zone markers only exist as models in the
  // (headless, on macOS) Gazebo world -- RViz has no way to show them on
  // its own. Publishing matching Markers here lets the one live view
  // available on this platform actually show what the robot is picking
  // and placing. transient_local so a late-joining RViz still gets them.
  void publishSceneMarkers()
  {
    static const std::map<std::string, std::array<float, 3>> object_colors{
      {"red_cube", {0.8f, 0.05f, 0.05f}},
      {"yellow_cube", {0.85f, 0.75f, 0.05f}},
      {"blue_cube", {0.05f, 0.15f, 0.8f}},
    };
    static const std::map<std::string, std::array<float, 3>> zone_colors{
      {"zone_a", {0.85f, 0.75f, 0.4f}},
      {"zone_b", {0.4f, 0.5f, 0.85f}},
      {"zone_c", {0.85f, 0.45f, 0.4f}},
    };

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "scene_markers", rclcpp::QoS(10).transient_local());

    visualization_msgs::msg::MarkerArray arr;
    int id = 0;
    std::string frame = move_group_->getPlanningFrame();

    for (const auto & [name, xy] : objects_) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame;
      m.header.stamp = now();
      m.ns = "objects";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = xy.first;
      m.pose.position.y = xy.second;
      m.pose.position.z = pick_descend_z_;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = m.scale.z = 0.04;
      auto c = object_colors.count(name) ? object_colors.at(name) : std::array<float, 3>{0.6f, 0.6f, 0.6f};
      m.color.r = c[0];
      m.color.g = c[1];
      m.color.b = c[2];
      m.color.a = 1.0f;
      arr.markers.push_back(m);
    }
    for (const auto & [name, xy] : zones_) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame;
      m.header.stamp = now();
      m.ns = "zones";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = xy.first;
      m.pose.position.y = xy.second;
      m.pose.position.z = 0.201;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = 0.08;
      m.scale.z = 0.002;
      auto c = zone_colors.count(name) ? zone_colors.at(name) : std::array<float, 3>{0.6f, 0.6f, 0.6f};
      m.color.r = c[0];
      m.color.g = c[1];
      m.color.b = c[2];
      m.color.a = 1.0f;
      arr.markers.push_back(m);
    }

    marker_pub_->publish(arr);
    RCLCPP_INFO(get_logger(), "Published %zu scene markers (objects + zones)", arr.markers.size());
  }

  bool moveNamedTarget(const std::string & name)
  {
    return withRetries(
      [this, &name]() {
        move_group_->setStartStateToCurrentState();
        move_group_->setNamedTarget(name);
        return move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
      },
      get_logger(), "moveNamedTarget(" + name + ")");
  }

  bool movePoseTarget(const geometry_msgs::msg::Pose & pose, const std::string & label)
  {
    return withRetries(
      [this, &pose]() {
        move_group_->setStartStateToCurrentState();
        move_group_->setPoseTarget(pose);
        return move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
      },
      get_logger(), label);
  }

  // `from` is passed explicitly (the pose the previous step already
  // commanded the arm to) purely so computeCartesianPath's two waypoints
  // are exactly right; the actual start configuration for planning comes
  // from setStartStateToCurrentState() (proven reliable elsewhere in this
  // file) rather than an eagerly-fetched RobotState -- calling
  // getCurrentState()/getCurrentPose() directly right after a move
  // finishes was observed to fail ("Failed to fetch current robot
  // state") on this setup, and a manual /joint_states subscription that
  // avoids that freshness check was itself observed to return a stale
  // snapshot, breaking the IK seed for the first waypoint.
  bool cartesianTo(
    const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & target,
    const std::string & label)
  {
    return withRetries(
      [this, &from, &target]() {
        std::vector<geometry_msgs::msg::Pose> waypoints{from, target};

        move_group_->setStartStateToCurrentState();
        moveit_msgs::msg::RobotTrajectory traj_msg;
        double fraction = move_group_->computeCartesianPath(waypoints, 0.005, 0.0, traj_msg);
        if (fraction < 0.95) return false;

        unwrapContinuousJoint(traj_msg.joint_trajectory, "wrist_3_joint");

        auto start_state = buildKnownState();
        robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(), planning_group_);
        rt.setRobotTrajectoryMsg(*start_state, traj_msg);
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        if (!totg.computeTimeStamps(rt, vel_scale_, accel_scale_)) return false;

        moveit_msgs::msg::RobotTrajectory retimed;
        rt.getRobotTrajectoryMsg(retimed);

        MoveGroupInterface::Plan plan;
        plan.trajectory_ = retimed;
        return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
      },
      get_logger(), label);
  }

  void handleHome(
    const std::shared_ptr<ur3_llm_control::srv::Home::Request>,
    std::shared_ptr<ur3_llm_control::srv::Home::Response> res)
  {
    RCLCPP_INFO(get_logger(), "SKILL home()");
    res->status = moveNamedTarget("up") ? "SUCCESS" : "FAILED";
  }

  void handlePick(
    const std::shared_ptr<ur3_llm_control::srv::Pick::Request> req,
    std::shared_ptr<ur3_llm_control::srv::Pick::Response> res)
  {
    RCLCPP_INFO(get_logger(), "SKILL pick(%s)", req->object.c_str());
    auto it = objects_.find(req->object);
    if (it == objects_.end()) {
      res->status = "INVALID_OBJECT";
      return;
    }
    if (!held_object_.empty()) {
      RCLCPP_WARN(get_logger(), "pick() rejected: already holding '%s'", held_object_.c_str());
      res->status = "FAILED";
      return;
    }

    double x = it->second.first, y = it->second.second;
    double approach_z = pick_descend_z_ + approach_height_;

    auto approach_pose = poseAt(x, y, approach_z);
    auto grasp_pose = poseAt(x, y, pick_descend_z_);

    if (!movePoseTarget(approach_pose, "pick: approach above " + req->object)) {
      res->status = "PLANNING_FAILED";
      return;
    }
    if (!cartesianTo(approach_pose, grasp_pose, "pick: descend to " + req->object)) {
      res->status = "PLANNING_FAILED";
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));  // simulated grasp
    if (!cartesianTo(grasp_pose, approach_pose, "pick: ascend with " + req->object)) {
      res->status = "PLANNING_FAILED";
      return;
    }

    held_object_ = req->object;
    res->status = "SUCCESS";
  }

  void handlePlace(
    const std::shared_ptr<ur3_llm_control::srv::Place::Request> req,
    std::shared_ptr<ur3_llm_control::srv::Place::Response> res)
  {
    RCLCPP_INFO(get_logger(), "SKILL place(%s, %s)", req->object.c_str(), req->zone.c_str());
    auto zit = zones_.find(req->zone);
    if (zit == zones_.end()) {
      res->status = "INVALID_ZONE";
      return;
    }
    if (held_object_ != req->object) {
      RCLCPP_WARN(
        get_logger(), "place() rejected: not holding '%s' (holding '%s')", req->object.c_str(),
        held_object_.empty() ? "<nothing>" : held_object_.c_str());
      res->status = "FAILED";
      return;
    }

    double x = zit->second.first, y = zit->second.second;
    double approach_z = place_descend_z_ + approach_height_;

    auto approach_pose = poseAt(x, y, approach_z);
    auto place_pose = poseAt(x, y, place_descend_z_);

    if (!movePoseTarget(approach_pose, "place: approach above " + req->zone)) {
      res->status = "PLANNING_FAILED";
      return;
    }
    if (!cartesianTo(approach_pose, place_pose, "place: descend into " + req->zone)) {
      res->status = "PLANNING_FAILED";
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));  // simulated release
    if (!cartesianTo(place_pose, approach_pose, "place: ascend from " + req->zone)) {
      res->status = "PLANNING_FAILED";
      return;
    }

    held_object_.clear();
    res->status = "SUCCESS";
  }

  std::shared_ptr<MoveGroupInterface> move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  std::string planning_group_{"ur_manipulator"};
  double approach_height_{approach_height_default_};
  double pick_descend_z_{pick_descend_z_default_};
  double place_descend_z_{place_descend_z_default_};
  double vel_scale_{vel_scale_default_};
  double accel_scale_{accel_scale_default_};
  static constexpr double approach_height_default_ = 0.12;
  static constexpr double pick_descend_z_default_ = 0.22;
  static constexpr double place_descend_z_default_ = 0.22;
  static constexpr double vel_scale_default_ = 0.3;
  static constexpr double accel_scale_default_ = 0.3;
  std::map<std::string, std::pair<double, double>> objects_, zones_;
  std::string held_object_;

  rclcpp::Service<ur3_llm_control::srv::Home>::SharedPtr home_srv_;
  rclcpp::Service<ur3_llm_control::srv::Pick>::SharedPtr pick_srv_;
  rclcpp::Service<ur3_llm_control::srv::Place>::SharedPtr place_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SkillServerNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  node->initMoveGroup();

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
