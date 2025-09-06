#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "traffic_planner/TrafficPlanner.hpp"

#include <vector>
#include <cmath>
#include <map>
#include <string>
#include <memory>

struct WaypointWithYaw {
    double x, y, yaw;
};

struct RobotData {
    std::vector<WaypointWithYaw> waypoints;
    size_t waypoint_index{0};
    geometry_msgs::msg::PoseStamped current_pose;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    rclcpp::TimerBase::SharedPtr timeout_timer;
    bool goal_sent{false};
    bool waypoint_reached{false};  // 현재 waypoint 도달 여부

    static constexpr double THRESHOLD = 0.04;
    static constexpr double THRESH_SQ = THRESHOLD * THRESHOLD;
};

class SynchronizedTrafficPlannerNode : public rclcpp::Node {
public:
    SynchronizedTrafficPlannerNode();

private:
    // 핵심 멤버
    std::unique_ptr<TrafficPlanner> planner_;
    std::map<int, RobotData> robots_;
    
    // 파라미터
    int num_robots_;
    double timeout_seconds_;
    double resolution_;
    bool use_disjoint_splitting_;
    
    // 동기화 관련
    bool synchronization_enabled_{true};
    rclcpp::TimerBase::SharedPtr sync_check_timer_;
    double sync_check_interval_{0.1};  // 100ms마다 동기화 체크
    
    // 초기화 함수들
    void initializeParameters();
    void initializeMap();
    void initializeRobots();
    void createRobotCommunication(int robot_id);
    
    // 시작점/목표점 생성
    std::vector<Position> generateStartPositions();
    std::vector<Position> generateGoalPositions();
    
    // 경로 계획 및 변환
    void planAndAssignPaths();
    std::vector<WaypointWithYaw> convertPathToWaypoints(const std::vector<Position>& path);
    
    // ROS2 통신 함수들
    void handlePose(int id, geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publishPath(int id);
    void sendGoalPose(int id);
    void startTimeout(int id);
    
    // 동기화 관련 함수들
    void checkSynchronization();
    bool allRobotsReachedWaypoint();
    void advanceAllWaypoints();
    void resetWaypointFlags();
    size_t getMaxWaypointIndex();
    bool allRobotsCompleted();
    
    // 디폴트 맵 생성
    std::vector<std::vector<bool>> createDefaultMap();
};