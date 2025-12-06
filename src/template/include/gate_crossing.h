#ifndef GATE_CROSSING_H
#define GATE_CROSSING_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>

class GateCrossing
{
public:
    GateCrossing(ros::NodeHandle& nh);
    ~GateCrossing();
    
    // 主控制函数
    bool update(float& vel_x, float& vel_y, float& vel_z, float& yaw_rate);
    
    // 控制接口
    void start();
    void stop();
    void reset();
    
    // 检测函数
    bool detectGate();
    
    // 状态查询
    bool isActive() const;
    bool isGateDetected() const;
    float getOffsetX() const;
    float getOffsetY() const;
    float getGateWidth() const;
    float getDistance() const;
    float getTotalDistance() const;
    
private:
    // ROS相关
    ros::NodeHandle& nh_;
    ros::Subscriber image_sub_;
    ros::Publisher debug_image_pub_;
    
    // 图像相关
    cv::Mat current_image_;
    bool has_image_;
    
    // 控制参数
    float kp_x_, ki_x_, kd_x_;
    float kp_y_, ki_y_, kd_y_;
    float base_speed_;
    float max_speed_;
    float min_speed_;
    float align_threshold_;
    float close_distance_;
    
    // 检测参数
    int black_threshold_;
    int min_area_;
    float min_aspect_ratio_;
    float max_aspect_ratio_;
    float min_gap_ratio_;
    float max_gap_ratio_;
    float max_height_diff_ratio_;
    float max_y_center_diff_ratio_;
    
    // 状态变量
    bool active_;
    bool gate_detected_;
    float offset_x_;
    float offset_y_;
    float gate_width_;
    float distance_;
    float total_distance_;
    float target_distance_;
    
    // PID控制变量
    float error_x_, error_y_;
    float integral_x_, integral_y_;
    float derivative_x_, derivative_y_;
    float last_error_x_, last_error_y_;
    
    // 回调函数
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    
    // 辅助函数
    float evaluatePillarPair(const cv::Rect& left, const cv::Rect& right, 
                            int image_width, int image_height);
    float estimateDistance(const cv::Rect& left, const cv::Rect& right,
                          int image_width, int image_height);
    void publishDebugImage();
};

#endif // GATE_CROSSING_H
