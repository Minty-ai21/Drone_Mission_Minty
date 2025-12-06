#ifndef COLLISION_AVOIDANCE_H
#define COLLISION_AVOIDANCE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>  //
#include <Eigen/Eigen>
#include <iostream>

class CollisionAvoidance
{
public:
    CollisionAvoidance(ros::NodeHandle& nh);
    ~CollisionAvoidance();
    
    bool initialize();
    bool calculateAvoidanceVelocity(float target_x, float target_y, float& output_vel_x, float& output_vel_y);
    bool isAvoidanceActive() const { return flag_collision_avoidance_; }  // 
    void printAvoidanceInfo();
    void printParameters();

private:
    // 回调函数
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    
    // 内部方法
    void calculateMinDistance();
    float saturationFunction(float data, float max_val);
    void coordinateRotation(float yaw_angle, float input[2], float output[2]);
    void avoidanceAlgorithm(float target_x, float target_y);
    void loadParameters();  // 参数加载方法
    
    // ROS相关
    ros::NodeHandle& nh_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber pos_sub_;
    
    // 避障参数
    float R_outside_, R_inside_;
    float p_R_, p_r_;
    float p_xy_;
    float vel_track_max_;
    float vel_collision_max_;
    float vel_sp_max_;
    int range_min_, range_max_;
    
    // 数据
    sensor_msgs::LaserScan laser_data_;
    geometry_msgs::PoseStamped current_position_;
    Eigen::Quaterniond attitude_quaternion_;
    Eigen::Vector3d euler_angles_;
    
    // 中间变量
    float distance_c_, angle_c_;
    float distance_cx_, distance_cy_;
    float vel_track_[2];
    float vel_collision_[2];
    float vel_sp_body_[2];
    float vel_sp_ENU_[2];
    
    // 状态标志 - 修改为基本bool类型
    bool flag_collision_avoidance_;  // 修改这行
    
    // 初始化标志
    bool initialized_;
    float init_position_x_, init_position_y_, init_position_z_;
    bool init_position_set_;
};

#endif // COLLISION_AVOIDANCE_H
