#include "traffic_planner/SynchronizedTrafficPlannerNode.hpp"
#include <chrono>

using namespace std::chrono_literals;

SynchronizedTrafficPlannerNode::SynchronizedTrafficPlannerNode() : Node("synchronized_traffic_planner_node") {
    RCLCPP_INFO(get_logger(), "Synchronized Traffic Path Planner Node 시작");
    
    initializeParameters();
    initializeMap();
    initializeRobots();
    
    // 동기화 체크 타이머 시작
    sync_check_timer_ = create_wall_timer(
        std::chrono::duration<double>(sync_check_interval_),
        [this]() { checkSynchronization(); }
    );
    
    RCLCPP_INFO(get_logger(), "%d개 로봇에 대한 Synchronized Traffic Path Planner 초기화 완료", num_robots_);
}

void SynchronizedTrafficPlannerNode::initializeParameters() {
    declare_parameter("num_robots", 2);
    declare_parameter("timeout_seconds", 800.0);
    declare_parameter("resolution", 0.1);
    declare_parameter("start_positions", std::vector<double>{});
    declare_parameter("goal_positions", std::vector<double>{});
    declare_parameter("use_disjoint_splitting", true);
    declare_parameter("synchronization_enabled", true);
    declare_parameter("sync_check_interval", 0.1);

    num_robots_ = get_parameter("num_robots").as_int();
    timeout_seconds_ = get_parameter("timeout_seconds").as_double();
    resolution_ = get_parameter("resolution").as_double();
    use_disjoint_splitting_ = get_parameter("use_disjoint_splitting").as_bool();
    synchronization_enabled_ = get_parameter("synchronization_enabled").as_bool();
    sync_check_interval_ = get_parameter("sync_check_interval").as_double();

    RCLCPP_INFO(get_logger(), "로봇 수: %d", num_robots_);
    RCLCPP_INFO(get_logger(), "타임아웃: %.1f초", timeout_seconds_);
    RCLCPP_INFO(get_logger(), "해상도: %.2fm", resolution_);
    RCLCPP_INFO(get_logger(), "Disjoint Splitting: %s", use_disjoint_splitting_ ? "활성화" : "비활성화");
    RCLCPP_INFO(get_logger(), "동기화 모드: %s", synchronization_enabled_ ? "활성화" : "비활성화");
    RCLCPP_INFO(get_logger(), "동기화 체크 주기: %.1fms", sync_check_interval_ * 1000);

    if (num_robots_ <= 0) {
        RCLCPP_ERROR(get_logger(), "로봇 수는 1 이상이어야 합니다");
        throw std::invalid_argument("로봇 수가 유효하지 않습니다");
    }
}

void SynchronizedTrafficPlannerNode::initializeMap() {
    auto map = createDefaultMap();
    planner_ = std::make_unique<TrafficPlanner>(map);
    RCLCPP_INFO(get_logger(), "동기화 교통 규칙 맵 초기화 완료");
}

void SynchronizedTrafficPlannerNode::initializeRobots() {
    auto starts = generateStartPositions();
    auto goals = generateGoalPositions();

    if (starts.size() != static_cast<size_t>(num_robots_) || 
        goals.size() != static_cast<size_t>(num_robots_)) {
        RCLCPP_ERROR(get_logger(), "시작점 또는 목표점 수가 로봇 수와 일치하지 않음");
        throw std::runtime_error("시작점/목표점 개수 불일치");
    }

    for (int i = 0; i < num_robots_; ++i) {
        RCLCPP_INFO(get_logger(), "로봇 %d: 시작(%d,%d) → 목표(%d,%d)", 
                    i+1, starts[i].first, starts[i].second, goals[i].first, goals[i].second);
    }

    planAndAssignPaths();

    for (int id = 1; id <= num_robots_; ++id) {
        createRobotCommunication(id);
        publishPath(id);
    }
}

void SynchronizedTrafficPlannerNode::planAndAssignPaths() {
    auto starts = generateStartPositions();
    auto goals = generateGoalPositions();

    std::vector<Constraint> custom_constraints = {
        {0, 3, { {5, 10} }, false},
    };
    
    std::string solver_type = use_disjoint_splitting_ ? "Synchronized CBS (Disjoint Splitting)" : "Synchronized CBS (Standard)";
    RCLCPP_INFO(get_logger(), "%s 솔버 시작 - 로봇 %d개", solver_type.c_str(), num_robots_);
    
    auto paths = planner_->planPaths(starts, goals, use_disjoint_splitting_, custom_constraints);

    if (paths.size() != static_cast<size_t>(num_robots_)) {
        RCLCPP_ERROR(get_logger(), "%s 경로 생성 실패", solver_type.c_str());
        throw std::runtime_error("경로 생성 실패");
    }

    RCLCPP_INFO(get_logger(), "%s 경로 생성 성공! %zu개 경로 생성됨", solver_type.c_str(), paths.size());

    for (int id = 1; id <= num_robots_; ++id) {
        auto& rd = robots_[id];
        rd.waypoints = convertPathToWaypoints(paths[id - 1]);
        RCLCPP_INFO(get_logger(), "[%d] Synchronized waypoints 생성 완료: %zu개", id, rd.waypoints.size());
    }
}

void SynchronizedTrafficPlannerNode::createRobotCommunication(int robot_id) {
    auto& rd = robots_[robot_id];

    std::string pose_topic = "/robot" + std::to_string(robot_id) + "/pose";
    rd.pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic, 10,
        [this, robot_id](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            handlePose(robot_id, msg);
        });

    std::string goal_topic = "/goalpose" + std::to_string(robot_id);
    rd.goal_pub = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);

    std::string path_topic = "/path" + std::to_string(robot_id);
    rd.path_pub = create_publisher<nav_msgs::msg::Path>(path_topic, 10);

    RCLCPP_INFO(get_logger(), "[%d] Synchronized ROS2 통신 설정 완료", robot_id);
}

void SynchronizedTrafficPlannerNode::handlePose(int id, geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    auto& rd = robots_[id];
    rd.current_pose = *msg;

    if (rd.waypoint_index >= rd.waypoints.size()) return;

    const auto& wp = rd.waypoints[rd.waypoint_index];
    double dx = rd.current_pose.pose.position.x - wp.x;
    double dy = rd.current_pose.pose.position.y - wp.y;

    if (!rd.goal_sent && !rd.waypoint_reached) {
        sendGoalPose(id);
        startTimeout(id);
        return;
    }

    // waypoint 도달 확인
    if (!rd.waypoint_reached && dx * dx + dy * dy < RobotData::THRESH_SQ) {
        rd.waypoint_reached = true;
        if (rd.timeout_timer) rd.timeout_timer->cancel();
        
        RCLCPP_INFO(get_logger(), "[%d] waypoint[%zu] 도달 완료! 다른 로봇들 대기 중...", 
                    id, rd.waypoint_index);
    }
}

void SynchronizedTrafficPlannerNode::checkSynchronization() {
    if (!synchronization_enabled_) return;
    if (allRobotsCompleted()) return;

    if (allRobotsReachedWaypoint()) {
        RCLCPP_INFO(get_logger(), "모든 로봇이 waypoint 도달! 동시에 다음 목표로 이동");
        advanceAllWaypoints();
    }
}

bool SynchronizedTrafficPlannerNode::allRobotsReachedWaypoint() {
    for (const auto& [id, rd] : robots_) {
        if (rd.waypoint_index >= rd.waypoints.size()) continue; // 완료된 로봇은 제외
        if (!rd.waypoint_reached) return false;
    }
    return true;
}

void SynchronizedTrafficPlannerNode::advanceAllWaypoints() {
    resetWaypointFlags();
    
    for (auto& [id, rd] : robots_) {
        if (rd.waypoint_index >= rd.waypoints.size()) continue; // 이미 완료된 로봇
        
        rd.waypoint_index++;
        rd.goal_sent = false;
        
        if (rd.waypoint_index < rd.waypoints.size()) {
            sendGoalPose(id);
            startTimeout(id);
        } else {
            RCLCPP_INFO(get_logger(), "[%d] 모든 waypoint 완료!", id);
        }
    }
    
    size_t current_waypoint = getMaxWaypointIndex();
    if (current_waypoint > 0) {
        RCLCPP_INFO(get_logger(), "=== 동기화된 waypoint %zu 진행 중 ===", current_waypoint);
    }
}

void SynchronizedTrafficPlannerNode::resetWaypointFlags() {
    for (auto& [id, rd] : robots_) {
        rd.waypoint_reached = false;
    }
}

size_t SynchronizedTrafficPlannerNode::getMaxWaypointIndex() {
    size_t max_index = 0;
    for (const auto& [id, rd] : robots_) {
        if (rd.waypoint_index > max_index) {
            max_index = rd.waypoint_index;
        }
    }
    return max_index;
}

bool SynchronizedTrafficPlannerNode::allRobotsCompleted() {
    for (const auto& [id, rd] : robots_) {
        if (rd.waypoint_index < rd.waypoints.size()) return false;
    }
    return true;
}

void SynchronizedTrafficPlannerNode::sendGoalPose(int id) {
    auto& rd = robots_[id];
    const auto& wp = rd.waypoints[rd.waypoint_index];

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = now();
    goal.pose.position.x = wp.x;
    goal.pose.position.y = wp.y;
    goal.pose.orientation.z = std::sin(wp.yaw / 2.0);
    goal.pose.orientation.w = std::cos(wp.yaw / 2.0);

    rd.goal_pub->publish(goal);
    rd.goal_sent = true;

    RCLCPP_INFO(get_logger(), "[%d] Synchronized goalpose → wp[%zu] (%.2f, %.2f)", 
        id, rd.waypoint_index, wp.x, wp.y);
}

void SynchronizedTrafficPlannerNode::startTimeout(int id) {
    auto& rd = robots_[id];
    if (rd.timeout_timer) rd.timeout_timer->cancel();

    rd.timeout_timer = create_wall_timer(
        std::chrono::duration<double>(timeout_seconds_),
        [this, id]() {
            RCLCPP_WARN(get_logger(), "[%d] %.0f초 timeout. waypoint 도달로 처리", id, timeout_seconds_);
            robots_[id].waypoint_reached = true;
            robots_[id].timeout_timer->cancel();
        });
}

std::vector<Position> SynchronizedTrafficPlannerNode::generateStartPositions() {
    auto custom_starts = get_parameter("start_positions").as_double_array();
    
    if (custom_starts.size() == static_cast<size_t>(num_robots_ * 2)) {
        std::vector<Position> starts;
        for (int i = 0; i < num_robots_; ++i) {
            int row = static_cast<int>(custom_starts[i * 2]);
            int col = static_cast<int>(custom_starts[i * 2 + 1]);
            starts.push_back({row, col});
        }
        RCLCPP_INFO(get_logger(), "커스텀 시작점 사용");
        return starts;
    }

    std::vector<Position> starts;
    int start_row = 2;
    int start_col = 19;
    
    for (int i = 0; i < num_robots_; ++i) {
        starts.push_back({start_row + i * 2, start_col});
    }
    
    RCLCPP_INFO(get_logger(), "기본 시작점 생성 완료 (%d개)", num_robots_);
    return starts;
}

std::vector<Position> SynchronizedTrafficPlannerNode::generateGoalPositions() {
    auto custom_goals = get_parameter("goal_positions").as_double_array();
    
    if (custom_goals.size() == static_cast<size_t>(num_robots_ * 2)) {
        std::vector<Position> goals;
        for (int i = 0; i < num_robots_; ++i) {
            int row = static_cast<int>(custom_goals[i * 2]);
            int col = static_cast<int>(custom_goals[i * 2 + 1]);
            goals.push_back({row, col});
        }
        RCLCPP_INFO(get_logger(), "커스텀 목표점 사용");
        return goals;
    }

    std::vector<Position> goals;
    int goal_row = 3;
    int goal_col = 11;
    
    for (int i = 0; i < num_robots_; ++i) {
        goals.push_back({goal_row + i * 2, goal_col});
    }
    
    RCLCPP_INFO(get_logger(), "기본 목표점 생성 완료 (%d개)", num_robots_);
    return goals;
}

std::vector<WaypointWithYaw> SynchronizedTrafficPlannerNode::convertPathToWaypoints(const std::vector<Position>& path) {
    std::vector<WaypointWithYaw> waypoints;
    
    for (const auto& cell : path) {
        double x = (cell.second - 1) * resolution_ + resolution_ / 2.0;
        double y = (cell.first - 1)  * resolution_ + resolution_ / 2.0;
        double yaw = M_PI / 2.0;
        waypoints.push_back({x, y, yaw});
    }
    
    return waypoints;
}

void SynchronizedTrafficPlannerNode::publishPath(int id) {
    auto& rd = robots_[id];
    
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = now();

    for (const auto& wp : rd.waypoints) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = now();
        pose.pose.position.x = wp.x;
        pose.pose.position.y = wp.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.z = std::sin(wp.yaw / 2.0);
        pose.pose.orientation.w = std::cos(wp.yaw / 2.0);
        
        path_msg.poses.push_back(pose);
    }

    rd.path_pub->publish(path_msg);
    RCLCPP_INFO(get_logger(), "[%d] Synchronized 경로 publish 완료 (%zu waypoints)", id, rd.waypoints.size());
}

std::vector<std::vector<bool>> SynchronizedTrafficPlannerNode::createDefaultMap() {
    return {
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,1,1,1,1,1,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,1,1,1,1,1,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,1,1,1,1,1,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
    };
}