/*
 * boustrophedon.cpp
 * Lawn-mower mission with home-as-origin transform,
 * robust setpoint publishing (8 Hz), and VIO-friendly arrival checks.
 */

 #include <chrono>
 #include <cmath>
 #include <memory>
 #include <string>
 #include <vector>
 #include <numeric>   // accumulate
 #include <tuple>     // tuple
 #include <algorithm>
 
 #include "rclcpp/rclcpp.hpp"
 #include "geometry_msgs/msg/pose_stamped.hpp"
 #include "visualization_msgs/msg/marker.hpp"
 #include "visualization_msgs/msg/marker_array.hpp"
 #include "mavros_msgs/srv/command_tol.hpp"
 #include "mavros_msgs/srv/command_bool.hpp"
 #include "mavros_msgs/srv/set_mode.hpp"
 #include "mavros_msgs/msg/state.hpp"
 
 using namespace std::chrono_literals;
 
 struct Waypoint {
   double x, y, z;
   bool land;
   std::string name;
 };
 
 enum class MissionState {
   WAIT_FOR_CONNECTION,
   SET_GUIDED_MODE,
   ARM_DRONE,
   WAIT_BEFORE_TAKEOFF,
   TAKEOFF,
   WAIT_AFTER_TAKEOFF,
   FLYING_TO_WAYPOINT,
   LANDING,
   MISSION_COMPLETE
 };
 
 class MissionManager : public rclcpp::Node {
 public:
   MissionManager()
   : Node("boustrophedon_node"),
     mission_state_(MissionState::WAIT_FOR_CONNECTION),
     current_wp_index_(0)
   {
     // Publishers
     pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 20);
     marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
     marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array", 10);
 
     // Subscribers
     pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
       "/mavros/vision_pose/pose", 10, std::bind(&MissionManager::poseCallback, this, std::placeholders::_1));
     state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
       "/mavros/state", 10, std::bind(&MissionManager::stateCallback, this, std::placeholders::_1));
 
     // Service clients
     takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
     land_client_    = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
     arm_client_     = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
     mode_client_    = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
 
     // Parameters
     this->declare_parameter<double>("rect_long", rect_long_);
     this->declare_parameter<double>("rect_short", rect_short_);
     this->declare_parameter<double>("lawn_gap", lawn_gap_);
     this->declare_parameter<double>("cruise_alt", cruise_alt_);
     this->declare_parameter<double>("home_x_from_origin", home_from_origin_x_);
     this->declare_parameter<double>("home_y_from_origin", home_from_origin_y_);
 
     this->get_parameter("rect_long", rect_long_);
     this->get_parameter("rect_short", rect_short_);
     this->get_parameter("lawn_gap", lawn_gap_);
     this->get_parameter("cruise_alt", cruise_alt_);
     this->get_parameter("home_x_from_origin", home_from_origin_x_);
     this->get_parameter("home_y_from_origin", home_from_origin_y_);
 
     // Build boustrophedon plan immediately (home-as-origin transform)
     buildLawnmowerPlan();   // fills mission_plan_ and publishes static markers
 
     // Mission loop at 8 Hz (125 ms)
     mission_timer_ = this->create_wall_timer(125ms, std::bind(&MissionManager::missionLoop, this));
 
     RCLCPP_INFO(this->get_logger(),
       "Boustrophedon node ready. L=%.2f, S=%.2f, gap=%.2f, alt=%.2f, home_from_origin=(%.2f, %.2f). "
       "Waypoints: %zu",
       rect_long_, rect_short_, lawn_gap_, cruise_alt_,
       home_from_origin_x_, home_from_origin_y_, mission_plan_.size());
   }
 
 private:
   // Publishers
   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
   rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
   rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
 
   // Subscribers
   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
   rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
 
   // Service clients
   rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_, land_client_;
   rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
   rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
 
   // State
   rclcpp::TimerBase::SharedPtr mission_timer_;
   geometry_msgs::msg::PoseStamped current_pose_;
   mavros_msgs::msg::State current_state_;
   std::vector<Waypoint> mission_plan_;
   size_t current_wp_index_;
   MissionState mission_state_;
   rclcpp::Time takeoff_start_time_, arm_time_, hover_time_;
   rclcpp::Time waypoint_start_time_, last_close_time_;
 
   // VIO arrival helpers
   bool close_to_waypoint_ = false;
   int consecutive_close_count_ = 0;
   std::vector<double> recent_distances_;
 
   // Params / settings
   double rect_long_ = 12.0;   // meters (arena long edge)
   double rect_short_ = 9.0;   // meters (arena short edge)
   double lawn_gap_ = 1.0;     // meters between passes
   double cruise_alt_ = 3.0;   // flight altitude
   double home_from_origin_x_ = 0.0; // Home offset from actual arena origin (diagram coords)
   double home_from_origin_y_ = 0.0;
 
   // Flags
   bool guided_sent_ = false, armed_sent_ = false, takeoff_sent_ = false;
 
   // Thresholds (tuned for VIO)
   const double APPROACH_THRESHOLD = 2.0;   // Start "approach" logic
   const double PRECISION_THRESHOLD = 0.6;  // Final precise arrival threshold
   const double POSITION_THRESHOLD = 0.5;   // Altitude error allowed to detect takeoff completion
   const double WAYPOINT_TIMEOUT = 25.0;    // Force progress if stuck
   const double CLOSE_CONFIRMATION_TIME = 1.0;  // Minimum time near target
   const int    MIN_CLOSE_COUNT = 8;        // ~1s at 8Hz
   const double VIO_NOISE_THRESHOLD = 0.25; // m (std dev)
   const int    DISTANCE_HISTORY_SIZE = 10;
   const double CONVERGENCE_RATE_THRESHOLD = 0.05; // m/s
 
   // ---------------- Callbacks ----------------
   void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
     current_pose_ = *msg;
   }
   void stateCallback(const mavros_msgs::msg::State::SharedPtr msg) {
     current_state_ = *msg;
   }
 
   // ---------------- Helpers ----------------
   void publishSetpoint(const Waypoint &wp) {
     geometry_msgs::msg::PoseStamped pose;
     pose.header.stamp = this->now();
     pose.header.frame_id = "map";          // keep consistent with VIO/map
     pose.pose.position.x = wp.x;
     pose.pose.position.y = wp.y;
     pose.pose.position.z = wp.z;
     pose.pose.orientation.w = 1.0;
     pose_pub_->publish(pose);
 
     // also publish a marker for the current target
     visualization_msgs::msg::Marker marker;
     marker.header.frame_id = "map";
     marker.header.stamp = this->now();
     marker.ns = "current_target";
     marker.id = 1;
     marker.type = visualization_msgs::msg::Marker::SPHERE;
     marker.action = visualization_msgs::msg::Marker::ADD;
     marker.pose.position.x = wp.x;
     marker.pose.position.y = wp.y;
     marker.pose.position.z = wp.z;
     marker.scale.x = 0.35;
     marker.scale.y = 0.35;
     marker.scale.z = 0.35;
     marker.color.r = 0.2f;
     marker.color.g = 1.0f;
     marker.color.b = 0.2f;
     marker.color.a = 1.0;
     marker_pub_->publish(marker);
   }
 
   double distanceTo(const Waypoint &wp) const {
     double dx = wp.x - current_pose_.pose.position.x;
     double dy = wp.y - current_pose_.pose.position.y;
     double dz = wp.z - current_pose_.pose.position.z;
     return std::sqrt(dx*dx + dy*dy + dz*dz);
   }
 
   bool isWaypointReached(const Waypoint &wp) {
     const double distance = distanceTo(wp);
 
     // rolling window
     recent_distances_.push_back(distance);
     if (recent_distances_.size() > DISTANCE_HISTORY_SIZE)
       recent_distances_.erase(recent_distances_.begin());
 
     // average distance
     double avg = std::accumulate(recent_distances_.begin(), recent_distances_.end(), 0.0)
                  / recent_distances_.size();
 
     // convergence rate (older 3 vs newer 3 samples)
     double convergence_rate = 0.0;
     if (recent_distances_.size() >= 6) {
       double old_avg = (recent_distances_[0] + recent_distances_[1] + recent_distances_[2]) / 3.0;
       double new_avg = (recent_distances_[recent_distances_.size()-3]
                       + recent_distances_[recent_distances_.size()-2]
                       + recent_distances_[recent_distances_.size()-1]) / 3.0;
       // samples are ~0.125s apart at 8 Hz; 3 samples ~0.375s
       convergence_rate = (old_avg - new_avg) / 0.375;
     }
 
     // std dev
     double var = 0.0;
     for (double d : recent_distances_) var += (d - avg) * (d - avg);
     var /= recent_distances_.size();
     double std_dev = std::sqrt(var);
 
     const bool within_approach   = avg < APPROACH_THRESHOLD;
     const bool within_precision  = avg < PRECISION_THRESHOLD;
     const bool vio_stable        = (std_dev < VIO_NOISE_THRESHOLD) && (recent_distances_.size() >= 5);
     const bool converging        = (convergence_rate > -CONVERGENCE_RATE_THRESHOLD);
     const bool stationary_near   = (std_dev < VIO_NOISE_THRESHOLD * 0.5) && within_precision;
 
     if (within_approach && !close_to_waypoint_) {
       close_to_waypoint_ = true;
       last_close_time_ = this->now();
       consecutive_close_count_ = 1;
       return false;
     }
 
     if (close_to_waypoint_) {
       consecutive_close_count_++;
 
       if (within_precision && vio_stable && converging) {
         const double time_close = (this->now() - last_close_time_).seconds();
         const bool time_ok  = time_close >= CLOSE_CONFIRMATION_TIME;
         const bool count_ok = consecutive_close_count_ >= MIN_CLOSE_COUNT;
 
         if (time_ok && count_ok) return true;
         if (stationary_near)     return true;
       }
     }
 
     // Reset if we moved away / unstable / diverging
     if (close_to_waypoint_ && (avg > APPROACH_THRESHOLD * 1.5
         || std_dev > VIO_NOISE_THRESHOLD * 3.0
         || convergence_rate < -CONVERGENCE_RATE_THRESHOLD * 2.0)) {
       close_to_waypoint_ = false;
       consecutive_close_count_ = 0;
     }
 
     return false;
   }
 
   // ---------------- Boustrophedon plan + static markers ----------------
   void buildLawnmowerPlan() {
     // Arena in actual-origin frame: A(0,0), B(L,0), C(L,S), D(0,S)
     // Transform to home-as-origin by subtracting (hx, hy)
     const double hx = home_from_origin_x_;
     const double hy = home_from_origin_y_;
     const double x0 = -hx;     // A'
     const double y0 = -hy;     // A'
     const double L  = rect_long_;
     const double S  = rect_short_;
     const double z  = cruise_alt_;
 
     const int rows = static_cast<int>(std::floor(S / std::max(1e-6, lawn_gap_))) + 1;
 
     mission_plan_.clear();
     mission_plan_.reserve(2 * rows + 2);
 
     // Home hover (index 0)
     mission_plan_.push_back({0.0, 0.0, z, false, "Home Hover"});
 
     for (int i = 0; i < rows; ++i) {
       double y = y0 + i * lawn_gap_;
       if (y > y0 + S) y = y0 + S;
 
       if ((i % 2) == 0) {
         // left -> right
         mission_plan_.push_back({x0,     y, z, false, "LM_A_" + std::to_string(i)});
         mission_plan_.push_back({x0 + L, y, z, false, "LM_B_" + std::to_string(i)});
       } else {
         // right -> left
         mission_plan_.push_back({x0 + L, y, z, false, "LM_B_" + std::to_string(i)});
         mission_plan_.push_back({x0,     y, z, false, "LM_A_" + std::to_string(i)});
       }
     }
 
     // Return and land
     mission_plan_.push_back({0.0, 0.0, z, true, "Land"});
 
     publishArenaAndPathMarkers(x0, y0, L, S, mission_plan_);
   }
 
   void publishArenaAndPathMarkers(double x0, double y0, double L, double S,
                                   const std::vector<Waypoint>& plan) {
     // Clear everything first
     visualization_msgs::msg::Marker clear;
     clear.header.frame_id = "map";
     clear.header.stamp = this->now();
     clear.ns = "lm";
     clear.id = 0;
     clear.action = visualization_msgs::msg::Marker::DELETEALL;
     marker_pub_->publish(clear);
 
     // Rectangle outline A'->B'->C'->D'->A'  (PINK)
     visualization_msgs::msg::Marker rect;
     rect.header.frame_id = "map";
     rect.header.stamp = this->now();
     rect.ns = "lm_rect";
     rect.id = 800;
     rect.type = visualization_msgs::msg::Marker::LINE_STRIP;
     rect.action = visualization_msgs::msg::Marker::ADD;
     rect.scale.x = 0.05;
     rect.color.r = 1.0; rect.color.g = 0.0; rect.color.b = 1.0; rect.color.a = 1.0; // PINK
 
     geometry_msgs::msg::Point p; p.z = 0.0;
     p.x = x0;      p.y = y0;       rect.points.push_back(p); // A'
     p.x = x0 + L;  p.y = y0;       rect.points.push_back(p); // B'
     p.x = x0 + L;  p.y = y0 + S;   rect.points.push_back(p); // C'
     p.x = x0;      p.y = y0 + S;   rect.points.push_back(p); // D'
     p.x = x0;      p.y = y0;       rect.points.push_back(p); // close
     marker_pub_->publish(rect);
 
     // Path line strip through all waypoints
     visualization_msgs::msg::Marker path;
     path.header.frame_id = "map";
     path.header.stamp = this->now();
     path.ns = "lm_path";
     path.id = 900;
     path.type = visualization_msgs::msg::Marker::LINE_STRIP;
     path.action = visualization_msgs::msg::Marker::ADD;
     path.scale.x = 0.06;
     path.color.r = 0.0; path.color.g = 0.6; path.color.b = 1.0; path.color.a = 1.0;
 
     for (const auto& wp : plan) {
       geometry_msgs::msg::Point q;
       q.x = wp.x; q.y = wp.y; q.z = wp.z;
       path.points.push_back(q);
     }
     marker_pub_->publish(path);
 
     // Waypoint spheres + labels
     visualization_msgs::msg::MarkerArray arr;
     int id_base = 100;
     for (size_t i = 0; i < plan.size(); ++i) {
       const auto& wp = plan[i];
 
       visualization_msgs::msg::Marker m;
       m.header.frame_id = "map";
       m.header.stamp = this->now();
       m.ns = "lm_wp";
       m.id = id_base + static_cast<int>(i);
       m.type = visualization_msgs::msg::Marker::SPHERE;
       m.action = visualization_msgs::msg::Marker::ADD;
       m.pose.position.x = wp.x;
       m.pose.position.y = wp.y;
       m.pose.position.z = wp.z;
       m.pose.orientation.w = 1.0;
       m.scale.x = 0.25; m.scale.y = 0.25; m.scale.z = 0.25;
 
       if (wp.land) { // landing marker
         m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.2; m.color.a = 1.0;
       } else if (i == 0) { // home - RED
         m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 1.0;
       } else {
         m.color.r = 0.2; m.color.g = 0.6; m.color.b = 1.0; m.color.a = 1.0;
       }
       arr.markers.push_back(m);
 
       visualization_msgs::msg::Marker t;
       t.header.frame_id = "map";
       t.header.stamp = this->now();
       t.ns = "lm_text";
       t.id = 1000 + static_cast<int>(i);
       t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
       t.action = visualization_msgs::msg::Marker::ADD;
       t.pose.position.x = wp.x;
       t.pose.position.y = wp.y;
       t.pose.position.z = wp.z + 0.5;
       t.scale.z = 0.25;
       t.color.r = 1.0; t.color.g = 1.0; t.color.b = 1.0; t.color.a = 0.95;
       t.text = wp.name;
       arr.markers.push_back(t);
     }
     marker_array_pub_->publish(arr);
   }
 
   // ---------------- Mission loop ----------------
   void missionLoop() {
     switch (mission_state_) {
       case MissionState::WAIT_FOR_CONNECTION:
         // Stream home hover setpoint while we wait (keeps offboard alive)
         publishSetpoint(mission_plan_[0]);
         if (current_state_.connected) {
           RCLCPP_INFO(this->get_logger(), "MAVROS connected. Setting GUIDED mode…");
           mission_state_ = MissionState::SET_GUIDED_MODE;
         }
         break;
 
       case MissionState::SET_GUIDED_MODE:
         publishSetpoint(mission_plan_[0]);
         if (!current_state_.guided && !guided_sent_) {
           auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
           req->base_mode = 0;
           req->custom_mode = "GUIDED";
           mode_client_->async_send_request(req);
           guided_sent_ = true;
         } else if (current_state_.guided) {
           mission_state_ = MissionState::ARM_DRONE;
           RCLCPP_INFO(this->get_logger(), "GUIDED set. Arming…");
         }
         break;
 
       case MissionState::ARM_DRONE:
         publishSetpoint(mission_plan_[0]);
         if (!current_state_.armed && !armed_sent_) {
           auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
           req->value = true;
           arm_client_->async_send_request(req);
           armed_sent_ = true;
           arm_time_ = this->now();
         } else if (current_state_.armed &&
                    (this->now() - arm_time_).seconds() > 5.0) {
           RCLCPP_INFO(this->get_logger(), "Armed. Hold before takeoff…");
           mission_state_ = MissionState::WAIT_BEFORE_TAKEOFF;
           takeoff_start_time_ = this->now();
         }
         break;
 
       case MissionState::WAIT_BEFORE_TAKEOFF:
         publishSetpoint(mission_plan_[0]);
         if ((this->now() - takeoff_start_time_).seconds() >= 5.0) {
           mission_state_ = MissionState::TAKEOFF;
         }
         break;
 
       case MissionState::TAKEOFF:
         publishSetpoint(mission_plan_[0]);
         if (!takeoff_sent_) {
           auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
           req->altitude = mission_plan_[0].z;
           req->latitude = 0.0;
           req->longitude = 0.0;
           req->yaw = 0.0;
           req->min_pitch = 0.0;
           takeoff_client_->async_send_request(req);
           takeoff_sent_ = true;
           takeoff_start_time_ = this->now();
           RCLCPP_INFO(this->get_logger(), "Takeoff command sent to %.2f m", req->altitude);
           }
         if ((this->now() - takeoff_start_time_).seconds() > 5.0 &&
             std::abs(current_pose_.pose.position.z - mission_plan_[0].z) < POSITION_THRESHOLD) {
           hover_time_ = this->now();
           mission_state_ = MissionState::WAIT_AFTER_TAKEOFF;
           RCLCPP_INFO(this->get_logger(), "Takeoff complete. Hovering…");
         }
         break;
 
       case MissionState::WAIT_AFTER_TAKEOFF:
         publishSetpoint(mission_plan_[0]);
         if ((this->now() - hover_time_).seconds() > 2.0) {
           mission_state_ = MissionState::FLYING_TO_WAYPOINT;
           current_wp_index_ = 1;                 // start at first pass
           waypoint_start_time_ = this->now();
           close_to_waypoint_ = false;
           consecutive_close_count_ = 0;
           recent_distances_.clear();
           RCLCPP_INFO(this->get_logger(), "Starting waypoint navigation.");
         }
         break;
 
       case MissionState::FLYING_TO_WAYPOINT:
         if (current_wp_index_ < mission_plan_.size()) {
           const auto &wp = mission_plan_[current_wp_index_];
           publishSetpoint(wp);
 
           const double current_distance = distanceTo(wp);
           const double elapsed = (this->now() - waypoint_start_time_).seconds();
 
           // Check arrival
           if (isWaypointReached(wp)) {
             RCLCPP_INFO(this->get_logger(), "Reached %s in %.1f s", wp.name.c_str(), elapsed);
 
             // move next
             current_wp_index_++;
             if (current_wp_index_ < mission_plan_.size()) {
               waypoint_start_time_ = this->now();
               close_to_waypoint_ = false;
               consecutive_close_count_ = 0;
               recent_distances_.clear();
             }
 
             // landing?
             if (wp.land) {
               mission_state_ = MissionState::LANDING;
               RCLCPP_INFO(this->get_logger(), "Landing sequence…");
             }
           }
           // Timeout fallback
           else if (elapsed > WAYPOINT_TIMEOUT) {
             RCLCPP_WARN(this->get_logger(),
               "Timeout at %s after %.1f s (dist %.2f m). Advancing.",
               wp.name.c_str(), elapsed, current_distance);
 
             current_wp_index_++;
             if (current_wp_index_ < mission_plan_.size()) {
               waypoint_start_time_ = this->now();
               close_to_waypoint_ = false;
               consecutive_close_count_ = 0;
               recent_distances_.clear();
             }
             if (wp.land) {
               mission_state_ = MissionState::LANDING;
               RCLCPP_INFO(this->get_logger(), "Landing sequence (timeout)…");
             }
           }
         } else {
           mission_state_ = MissionState::MISSION_COMPLETE;
         }
         break;
 
       case MissionState::LANDING:
         RCLCPP_INFO(this->get_logger(), "Initiating landing…");
         land_client_->async_send_request(std::make_shared<mavros_msgs::srv::CommandTOL::Request>());
         mission_state_ = MissionState::MISSION_COMPLETE;
         break;
 
       case MissionState::MISSION_COMPLETE:
         // Do nothing, keep timer alive for setpoint stream safety if needed
         break;
     }
   }
 };
 
 int main(int argc, char **argv) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<MissionManager>());
   rclcpp::shutdown();
   return 0;
 }
 