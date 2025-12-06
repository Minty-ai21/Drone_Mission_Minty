#include "collision_avoidance.h"
#include <tf/transform_datatypes.h>
#include <iostream>

CollisionAvoidance::CollisionAvoidance(ros::NodeHandle& nh) 
    : nh_(nh), initialized_(false), init_position_set_(false), flag_collision_avoidance_(false)  // 初始化flag
{
    // 初始化数组
    for(int i = 0; i < 2; i++) {
        vel_track_[i] = 0;
        vel_collision_[i] = 0;
        vel_sp_body_[i] = 0;
        vel_sp_ENU_[i] = 0;
    }
}

CollisionAvoidance::~CollisionAvoidance()
{
    // 清理资源
}

bool CollisionAvoidance::initialize()
{
    // 1. 读取所有避障参数
    loadParameters();
    
    // 2. 订阅话题
    lidar_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/laser/scan", 10, 
                       &CollisionAvoidance::lidarCallback, this);
    pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10,
                       &CollisionAvoidance::positionCallback, this);
    
    // 3. 等待初始位置数据
    ros::Rate rate(10);
    int wait_count = 0;
    while(ros::ok() && !init_position_set_ && wait_count < 50) {
        ros::spinOnce();
        rate.sleep();
        wait_count++;
    }
    
    if(!init_position_set_) {
        ROS_ERROR("Failed to get initial position within 5 seconds");
        return false;
    }
    
    initialized_ = true;
    ROS_INFO("Collision avoidance module initialized successfully");
    printParameters();
    
    return true;
}

void CollisionAvoidance::loadParameters()
{
    // 基本避障参数
    nh_.param<float>("R_outside", R_outside_, 2.0f);
    nh_.param<float>("R_inside", R_inside_, 1.0f);
    nh_.param<float>("p_xy", p_xy_, 0.5f);
    nh_.param<float>("vel_track_max", vel_track_max_, 0.5f);
    nh_.param<float>("p_R", p_R_, 0.5f);
    nh_.param<float>("p_r", p_r_, 0.5f);
    nh_.param<float>("vel_collision_max", vel_collision_max_, 0.5f);
    nh_.param<float>("vel_sp_max", vel_sp_max_, 1.0f);
    nh_.param<int>("range_min", range_min_, 0);
    nh_.param<int>("range_max", range_max_, 359);
    
    // 飞行高度参数
    float fly_height, height_square;
    nh_.param<float>("fly_height", fly_height, 0.5f);
    nh_.param<float>("height_square", height_square, 0.5f);
}

bool CollisionAvoidance::calculateAvoidanceVelocity(float target_x, float target_y, 
                                                   float& output_vel_x, float& output_vel_y)
{
    if(!initialized_) {
        ROS_ERROR("Collision avoidance module not initialized");
        return false;
    }
    
    // 处理回调
    ros::spinOnce();
    
    // 执行避障算法
    avoidanceAlgorithm(target_x, target_y);
    
    // 输出结果
    output_vel_x = vel_sp_ENU_[0];
    output_vel_y = vel_sp_ENU_[1];
    
    return true;
}

void CollisionAvoidance::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    sensor_msgs::LaserScan laser_tmp = *scan;
    laser_data_ = *scan;
    
    int count = laser_data_.ranges.size();
    
    // 剔除无效数据
    for(int i = 0; i < count; i++) {
        if(isinf(laser_tmp.ranges[i])) {
            if(i == 0) {
                laser_tmp.ranges[i] = laser_tmp.ranges[count-1];
            } else {
                laser_tmp.ranges[i] = laser_tmp.ranges[i-1];
            }
        }
    }
    
    // 重新组织数据顺序（前向为0度）
    for(int i = 0; i < count; i++) {
        if(i + 180 > 359) {
            laser_data_.ranges[i] = laser_tmp.ranges[i-180];
        } else {
            laser_data_.ranges[i] = laser_tmp.ranges[i+180];
        }
    }
    
    // 计算最小距离
    calculateMinDistance();
}

void CollisionAvoidance::positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_position_ = *msg;
    
    // 记录初始位置
    if(!init_position_set_) {
        init_position_x_ = msg->pose.position.x;
        init_position_y_ = msg->pose.position.y;
        init_position_z_ = msg->pose.position.z;
        init_position_set_ = true;
    }
    
    // 获取姿态
    attitude_quaternion_ = Eigen::Quaterniond(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z
    );
    
    // 转换为欧拉角
    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(msg->pose.orientation, tf_quat);
    tf::Matrix3x3(tf_quat).getRPY(euler_angles_[0], euler_angles_[1], euler_angles_[2]);
}

void CollisionAvoidance::calculateMinDistance()
{
    distance_c_ = laser_data_.ranges[range_min_];
    angle_c_ = 0;
    
    for(int i = range_min_; i <= range_max_; i++) {
        if(laser_data_.ranges[i] < distance_c_ && laser_data_.ranges[i] > 0.1) {
            distance_c_ = laser_data_.ranges[i];
            angle_c_ = i;
        }
    }
}

float CollisionAvoidance::saturationFunction(float data, float max_val)
{
    if(fabs(data) > max_val) {
        return (data > 0) ? max_val : -max_val;
    }
    return data;
}

void CollisionAvoidance::coordinateRotation(float yaw_angle, float input[2], float output[2])
{
    output[0] = input[0] * cos(yaw_angle) - input[1] * sin(yaw_angle);
    output[1] = input[0] * sin(yaw_angle) + input[1] * cos(yaw_angle);
}

void CollisionAvoidance::avoidanceAlgorithm(float target_x, float target_y)
{
    // ========== 1. 避障激活判断 ==========
    flag_collision_avoidance_ = (distance_c_ < R_outside_ && distance_c_ > 0.1);
    
    // 如果没有障碍物威胁，只计算追踪速度
    if (!flag_collision_avoidance_) {
        // 计算相对目标位置（考虑起飞点偏移）
        float relative_target_x = target_x + init_position_x_;
        float relative_target_y = target_y + init_position_y_;
        
        // 计算追踪速度
        vel_track_[0] = p_xy_ * (relative_target_x - current_position_.pose.position.x);
        vel_track_[1] = p_xy_ * (relative_target_y - current_position_.pose.position.y);
        
        // 追踪速度限幅
        for(int i = 0; i < 2; i++) {
            vel_track_[i] = saturationFunction(vel_track_[i], vel_track_max_);
        }
        
        // 避障速度为零
        vel_collision_[0] = 0;
        vel_collision_[1] = 0;
        
        // 直接合成速度
        vel_sp_body_[0] = vel_track_[0];
        vel_sp_body_[1] = vel_track_[1];
        
        // 坐标系转换
        coordinateRotation(euler_angles_[2], vel_sp_body_, vel_sp_ENU_);
        return;
    }
    
    // ========== 2. 有障碍物威胁，执行完整避障算法 ==========
    
    // 计算相对目标位置
    float relative_target_x = target_x + init_position_x_;
    float relative_target_y = target_y + init_position_y_;
    
    // 计算到目标的向量
    float dx_target = relative_target_x - current_position_.pose.position.x;
    float dy_target = relative_target_y - current_position_.pose.position.y;
    float target_dist = sqrt(dx_target*dx_target + dy_target*dy_target);
    
    // 归一化目标方向
    float target_dir_x = 0, target_dir_y = 0;
    if(target_dist > 0.001) {
        target_dir_x = dx_target / target_dist;
        target_dir_y = dy_target / target_dist;
    }
    
    // 计算追踪速度
    vel_track_[0] = p_xy_ * dx_target;
    vel_track_[1] = p_xy_ * dy_target;
    
    // 追踪速度限幅
    for(int i = 0; i < 2; i++) {
        vel_track_[i] = saturationFunction(vel_track_[i], vel_track_max_);
    }
    
    // 初始化避障速度
    vel_collision_[0] = 0;
    vel_collision_[1] = 0;
    
    // ========== 3. 切向力避障策略 ==========
    if(flag_collision_avoidance_) {
        // 计算障碍物分量
        distance_cx_ = distance_c_ * cos(angle_c_ * M_PI / 180.0);
        distance_cy_ = distance_c_ * sin(angle_c_ * M_PI / 180.0);
        
        // 计算障碍物方向向量（从无人机指向障碍物）
        float obstacle_dir_x = distance_cx_ / distance_c_;
        float obstacle_dir_y = distance_cy_ / distance_c_;
        
        // 计算目标方向与障碍物方向的夹角
        float dot_product = target_dir_x * obstacle_dir_x + target_dir_y * obstacle_dir_y;
        float cross_product = target_dir_x * obstacle_dir_y - target_dir_y * obstacle_dir_x;
        float angle_between = acos(fmax(-1.0f, fmin(1.0f, dot_product)));
        
        // 只对前方障碍物避障（夹角小于90度）
        if(angle_between < M_PI/2.0) {
            float F_normal = 0;  // 法向排斥力
            float F_tangent = 0; // 切向绕行力
            
            // 分层避障策略
            if(distance_c_ > R_inside_ && distance_c_ <= R_outside_) {
                // 内外圈之间：轻度避障
                F_normal = p_R_ * (R_outside_ - distance_c_);
                F_tangent = 0.4 * F_normal;  // 切向力为法向力的40%
            } 
            else if(distance_c_ <= R_inside_) {
                // 内圈内：强力避障
                F_normal = p_R_ * (R_outside_ - R_inside_) + p_r_ * (R_inside_ - distance_c_);
                F_tangent = 0.6 * F_normal;  // 切向力为法向力的60%
            }
            
            // ========== 计算法向排斥力 ==========
            // 法向力方向：远离障碍物（与障碍物方向相反）
            float F_normal_x = -F_normal * obstacle_dir_x;
            float F_normal_y = -F_normal * obstacle_dir_y;
            
            // ========== 计算切向绕行力 ==========
            // 选择绕行方向：总是选择能更快到达目标的方向
            float tangent_dir_x, tangent_dir_y;
            
            if(cross_product > 0) {
                // 目标在障碍物右侧，向右绕行（顺时针90度）
                tangent_dir_x = obstacle_dir_y;
                tangent_dir_y = -obstacle_dir_x;
            } else {
                // 目标在障碍物左侧，向左绕行（逆时针90度）
                tangent_dir_x = -obstacle_dir_y;
                tangent_dir_y = obstacle_dir_x;
            }
            
            float F_tangent_x = F_tangent * tangent_dir_x;
            float F_tangent_y = F_tangent * tangent_dir_y;
            
            // ========== 合成避障力 ==========
            // 根据障碍物位置调整增益
            float avoidance_gain = 1.0f;
            
            if(dot_product > 0.7f) {
                // 障碍物在前方，增强避障
                avoidance_gain = 1.2f;
            } else if(dot_product < -0.7f) {
                // 障碍物在后方，减弱避障
                avoidance_gain = 0.3f;
            }
            
            vel_collision_[0] = (F_normal_x + F_tangent_x) * avoidance_gain;
            vel_collision_[1] = (F_normal_y + F_tangent_y) * avoidance_gain;
            
            // ========== 早避障策略 ==========
            // 在外圈就主动避障，添加目标导向分量
            if(distance_c_ < R_outside_ * 0.8f) {
                float target_influence = 0.2f * (1.0f - distance_c_ / R_outside_);
                vel_collision_[0] += target_influence * target_dir_x * vel_track_max_;
                vel_collision_[1] += target_influence * target_dir_y * vel_track_max_;
            }
            
            // 避障速度限幅
            for(int i = 0; i < 2; i++) {
                vel_collision_[i] = saturationFunction(vel_collision_[i], vel_collision_max_);
            }
            
            // ========== 调试信息输出 ==========
            static int debug_counter = 0;
            debug_counter++;
            if(debug_counter % 50 == 0) {  // 每2.5秒输出一次（20Hz * 50）
                std::cout << "======= 切向力避障详情 =======" << std::endl;
                std::cout << "障碍物距离: " << distance_c_ << "m, 角度: " << angle_c_ << "度" << std::endl;
                std::cout << "目标-障碍物夹角: " << angle_between * 180.0/M_PI << "度" << std::endl;
                std::cout << "绕行方向: " << (cross_product > 0 ? "右侧" : "左侧") << std::endl;
                std::cout << "法向力: " << F_normal << ", 切向力: " << F_tangent << std::endl;
                std::cout << "合成避障速度: (" << vel_collision_[0] << ", " << vel_collision_[1] << ")" << std::endl;
                std::cout << "=============================" << std::endl;
                debug_counter = 0;
            }
        }
    }
    
    // ========== 4. 速度合成 ==========
    vel_sp_body_[0] = vel_track_[0] + vel_collision_[0];
    vel_sp_body_[1] = vel_track_[1] + vel_collision_[1];
    
    // 总速度限幅
    for(int i = 0; i < 2; i++) {
        vel_sp_body_[i] = saturationFunction(vel_sp_body_[i], vel_sp_max_);
    }
    
    // ========== 5. 坐标系转换 ==========
    coordinateRotation(euler_angles_[2], vel_sp_body_, vel_sp_ENU_);
}

void CollisionAvoidance::printAvoidanceInfo()
{
    std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Collision Avoidance Info <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    std::cout << "Min Distance: " << distance_c_ << " [m] at angle: " << angle_c_ << " [deg]" << std::endl;
    std::cout << "Obstacle XY: (" << distance_cx_ << ", " << distance_cy_ << ") [m]" << std::endl;
    std::cout << "Avoidance Active: " << (flag_collision_avoidance_ ? "YES" : "NO") << std::endl;  // 修改这行
    std::cout << "Track Velocity: (" << vel_track_[0] << ", " << vel_track_[1] << ") [m/s]" << std::endl;
    std::cout << "Avoidance Velocity: (" << vel_collision_[0] << ", " << vel_collision_[1] << ") [m/s]" << std::endl;
    std::cout << "Total Velocity (ENU): (" << vel_sp_ENU_[0] << ", " << vel_sp_ENU_[1] << ") [m/s]" << std::endl;
    // 当前位置信息
    std::cout << "当前位置: ("<< current_position_.pose.position.x << ", " << current_position_.pose.position.y << ") [m]"<< std::endl;
    std::cout << "================================================================================" << std::endl;
}

void CollisionAvoidance::printParameters()
{
    std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>> Avoidance Parameters <<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    std::cout << "R_outside: " << R_outside_ << " [m]" << std::endl;
    std::cout << "R_inside: " << R_inside_ << " [m]" << std::endl;
    std::cout << "p_R: " << p_R_ << std::endl;
    std::cout << "p_r: " << p_r_ << std::endl;
    std::cout << "p_xy: " << p_xy_ << std::endl;
    std::cout << "vel_track_max: " << vel_track_max_ << " [m/s]" << std::endl;
    std::cout << "vel_collision_max: " << vel_collision_max_ << " [m/s]" << std::endl;
    std::cout << "vel_sp_max: " << vel_sp_max_ << " [m/s]" << std::endl;
    std::cout << "range: [" << range_min_ << ", " << range_max_ << "] degrees" << std::endl;
}
