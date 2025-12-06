#include "gate_crossing.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <algorithm>
#include <cmath>

GateCrossing::GateCrossing(ros::NodeHandle& nh) 
    : nh_(nh),
      active_(false),
      gate_detected_(false),
      has_image_(false),
      offset_x_(0.0f),
      offset_y_(0.0f),
      gate_width_(0.0f),
      distance_(0.0f),
      total_distance_(0.0f),
      target_distance_(4.0f)
{
    // 加载控制参数
    nh_.param<float>("gate_kp_x", kp_x_, 0.4f);
    nh_.param<float>("gate_ki_x", ki_x_, 0.01f);
    nh_.param<float>("gate_kd_x", kd_x_, 0.05f);
    nh_.param<float>("gate_kp_y", kp_y_, 0.3f);
    nh_.param<float>("gate_ki_y", ki_y_, 0.008f);
    nh_.param<float>("gate_kd_y", kd_y_, 0.03f);
    nh_.param<float>("gate_base_speed", base_speed_, 0.2f);
    nh_.param<float>("gate_max_speed", max_speed_, 0.3f);
    nh_.param<float>("gate_min_speed", min_speed_, 0.1f);
    nh_.param<float>("gate_align_thresh", align_threshold_, 0.1f);
    nh_.param<float>("gate_close_dist", close_distance_, 2.0f);
    
    // 加载检测参数
    nh_.param<int>("gate_black_thresh", black_threshold_, 50);
    nh_.param<int>("gate_min_area", min_area_, 1000);
    nh_.param<float>("gate_min_aspect_ratio", min_aspect_ratio_, 2.0f);
    nh_.param<float>("gate_max_aspect_ratio", max_aspect_ratio_, 20.0f);
    nh_.param<float>("gate_min_gap_ratio", min_gap_ratio_, 0.1f);
    nh_.param<float>("gate_max_gap_ratio", max_gap_ratio_, 5.0f);
    nh_.param<float>("gate_max_height_diff_ratio", max_height_diff_ratio_, 0.3f);
    nh_.param<float>("gate_max_y_center_diff_ratio", max_y_center_diff_ratio_, 0.3f);
    
    // PID控制器初始化
    error_x_ = 0.0f;
    error_y_ = 0.0f;
    integral_x_ = 0.0f;
    integral_y_ = 0.0f;
    derivative_x_ = 0.0f;
    derivative_y_ = 0.0f;
    last_error_x_ = 0.0f;
    last_error_y_ = 0.0f;
    
    // 订阅相机话题
    image_sub_ = nh_.subscribe<sensor_msgs::Image>("/camera/image_raw", 1, 
                                                   &GateCrossing::imageCallback, this);
    
    // 调试图像发布
    debug_image_pub_ = nh_.advertise<sensor_msgs::Image>("gate_debug_image", 1);
    
    ROS_INFO("GateCrossing initialized with parameters:");
    ROS_INFO("  kp_x: %.2f, ki_x: %.2f, kd_x: %.2f", kp_x_, ki_x_, kd_x_);
    ROS_INFO("  kp_y: %.2f, ki_y: %.2f, kd_y: %.2f", kp_y_, ki_y_, kd_y_);
    ROS_INFO("  base_speed: %.2f, max_speed: %.2f", base_speed_, max_speed_);
    ROS_INFO("  black_threshold: %d, min_area: %d", black_threshold_, min_area_);
}

GateCrossing::~GateCrossing()
{
    // 清理资源
}

void GateCrossing::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, 
            sensor_msgs::image_encodings::BGR8);
        current_image_ = cv_ptr->image.clone();
        has_image_ = true;
        
        // 可选：发布调试图像
        publishDebugImage();
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

bool GateCrossing::detectGate()
{
    if (!has_image_ || current_image_.empty()) {
        gate_detected_ = false;
        return false;
    }
    
    cv::Mat image = current_image_.clone();
    int width = image.cols;
    int height = image.rows;
    
    // 1. 转为灰度图
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    
    // 2. 二值化检测黑色
    cv::Mat binary;
    cv::threshold(gray, binary, black_threshold_, 255, cv::THRESH_BINARY_INV);
    
    // 3. 形态学处理
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
    
    // 4. 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // 5. 寻找可能的门柱
    std::vector<cv::Rect> pillars;
    
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < min_area_) continue;
        
        cv::Rect rect = cv::boundingRect(contour);
        float aspect = (float)rect.height / rect.width;
        
        if (aspect > min_aspect_ratio_ && aspect < max_aspect_ratio_) {  // 垂直矩形
            pillars.push_back(rect);
        }
    }
    
    // 6. 寻找最佳的门柱配对
    if (pillars.size() >= 2) {
        // 按X坐标排序
        std::sort(pillars.begin(), pillars.end(),
            [](const cv::Rect& a, const cv::Rect& b) { 
                return a.x < b.x; 
            });
        
        float best_score = 0.0f;
        std::pair<cv::Rect, cv::Rect> best_pair;
        bool found_pair = false;
        
        for (size_t i = 0; i < pillars.size(); ++i) {
            for (size_t j = i + 1; j < pillars.size(); ++j) {
                cv::Rect left = pillars[i];
                cv::Rect right = pillars[j];
                
                if (left.x > right.x) std::swap(left, right);
                
                // 计算配对得分
                float score = evaluatePillarPair(left, right, width, height);
                
                if (score > best_score && score > 0.5f) {  // 得分阈值
                    best_score = score;
                    best_pair = std::make_pair(left, right);
                    found_pair = true;
                }
            }
        }
        
        // 7. 检查是否找到合适的门
        if (found_pair) {
            cv::Rect left = best_pair.first;
            cv::Rect right = best_pair.second;
            
            // 找到门
            float center_x = (left.x + left.width/2.0f + right.x + right.width/2.0f) / 2.0f;
            float center_y = (left.y + left.height/2.0f + right.y + right.height/2.0f) / 2.0f;
            
            // 中心偏移
            offset_x_ = 2.0f * (center_x / width) - 1.0f;  // -1~1
            offset_y_ = 2.0f * (center_y / height) - 1.0f; // -1~1
            
            // 门宽度
            gate_width_ = (right.x + right.width) - left.x;
            
            // 距离估计
            distance_ = estimateDistance(left, right, width, height);
            
            // 计算门的大小和间隙
            float pillar_width = (left.width + right.width) / 2.0f;
            float gap = right.x - (left.x + left.width);
            
            gate_detected_ = true;
            
            // 输出检测信息
            static int info_counter = 0;
            if (info_counter++ % 20 == 0) {
                ROS_INFO("Gate detected: width=%.1fpix, gap=%.1fpix, distance=%.2fm, score=%.2f", 
                        gate_width_, gap, distance_, best_score);
                info_counter = 0;
            }
            
            return true;
        }
    }
    
    gate_detected_ = false;
    return false;
}

float GateCrossing::evaluatePillarPair(const cv::Rect& left, const cv::Rect& right, 
                                       int image_width, int image_height)
{
    float score = 1.0f;
    
    // 1. 计算高度一致性
    float height_diff_ratio = fabs(left.height - right.height) / 
                             std::max(left.height, right.height);
    if (height_diff_ratio > max_height_diff_ratio_) {
        score *= 0.5f;  // 高度差异过大，降低得分
    } else {
        score *= (1.0f - height_diff_ratio / max_height_diff_ratio_);
    }
    
    // 2. 计算Y中心对齐
    float left_center_y = left.y + left.height / 2.0f;
    float right_center_y = right.y + right.height / 2.0f;
    float y_center_diff = fabs(left_center_y - right_center_y) / image_height;
    
    if (y_center_diff > max_y_center_diff_ratio_) {
        score *= 0.3f;  // 垂直不对齐，大幅降低得分
    } else {
        score *= (1.0f - y_center_diff / max_y_center_diff_ratio_);
    }
    
    // 3. 计算间隙合理性
    float gap = right.x - (left.x + left.width);
    if (gap <= 0) {
        return 0.0f;  // 没有间隙或重叠，无效
    }
    
    float avg_width = (left.width + right.width) / 2.0f;
    float gap_width_ratio = gap / avg_width;
    
    // 间隙应该在宽度的0.1-5倍之间
    if (gap_width_ratio < min_gap_ratio_ || gap_width_ratio > max_gap_ratio_) {
        score *= 0.2f;  // 间隙不合理，降低得分
    } else {
        // 间隙越接近典型值得分越高
        float ideal_gap_ratio = 1.0f;  // 间隙与宽度比约为1:1
        float gap_score = 1.0f - fabs(gap_width_ratio - ideal_gap_ratio) / ideal_gap_ratio;
        gap_score = std::max(0.0f, std::min(1.0f, gap_score));
        score *= (0.5f + 0.5f * gap_score);
    }
    
    // 4. 宽度一致性
    float width_diff_ratio = fabs(left.width - right.width) / 
                           std::max(left.width, right.width);
    score *= (1.0f - width_diff_ratio * 0.5f);
    
    return score;
}

float GateCrossing::estimateDistance(const cv::Rect& left, const cv::Rect& right,
                                   int image_width, int image_height)
{
    // 改进的距离估计，考虑门柱高度和宽度
    float avg_height = (left.height + right.height) / 2.0f;
    float avg_width = (left.width + right.width) / 2.0f;
    
    // 假设实际门高3m，焦距已知
    float focal_length_pix = 500.0f;  // 假设焦距
    float real_height = 3.0f;  // 实际门高3m
    
    // 距离 = (实际高度 * 焦距) / 图像高度
    float distance_from_height = (real_height * focal_length_pix) / avg_height;
    
    // 也可以从宽度估算
    float gate_width_pix = (right.x + right.width) - left.x;
    float real_gate_width = 0.74f;  // 实际门总宽0.74m
    float distance_from_width = (real_gate_width * focal_length_pix) / gate_width_pix;
    
    // 返回平均值
    return (distance_from_height + distance_from_width) / 2.0f;
}

bool GateCrossing::update(float& vel_x, float& vel_y, float& vel_z, float& yaw_rate)
{
    if (!active_) 
    {
        vel_x = 0.0f;
        vel_y = 0.0f;
        vel_z = 0.0f;
        yaw_rate = 0.0f;
        return false;
    }
    
    // 1. 检测门
    detectGate();
    
    // 2. 如果没有检测到门，旋转搜索
    if (!gate_detected_) 
    {
        vel_x = 0.0f;       // 不前进
        vel_y = 0.0f;       // 不左右
        vel_z = 0.0f;       // 不上下
        yaw_rate = 0.3f;    // 旋转搜索
        
        // 输出搜索状态
        static int search_counter = 0;
        if (search_counter++ % 20 == 0) 
        {
            ROS_WARN("GateCrossing: 未检测到门，正在搜索...");
            search_counter = 0;
        }
        return true;
    }
    
    // 3. 检测到门，进行PID控制
    // 计算误差
    error_x_ = offset_x_;
    error_y_ = offset_y_;
    
    // 积分项
    integral_x_ += error_x_;
    integral_y_ += error_y_;
    
    // 微分项
    derivative_x_ = error_x_ - last_error_x_;
    derivative_y_ = error_y_ - last_error_y_;
    
    // 保存误差
    last_error_x_ = error_x_;
    last_error_y_ = error_y_;
    
    // 计算控制输出
    float target_vel_x = 0.0f;
    float target_vel_y = 0.0f;
    float target_vel_z = 0.0f;
    
    // 自适应前进速度：距离越近，速度越慢
    float speed_factor = std::min(1.0f, distance_ / 5.0f);
    target_vel_x = base_speed_ * speed_factor;
    
    // 水平控制
    if (fabs(offset_x_) > 0.05f)  // 偏移超过5%
    {
        target_vel_y = -(kp_x_ * error_x_ + ki_x_ * integral_x_ + kd_x_ * derivative_x_);
    }
    
    // 垂直控制
    if (fabs(offset_y_) > 0.05f)  // 偏移超过5%
    {
        target_vel_z = -(kp_y_ * error_y_ + ki_y_ * integral_y_ + kd_y_ * derivative_y_);
    }
    
    // 4. 限制速度
    vel_x = std::min(std::max(target_vel_x, min_speed_), max_speed_);
    vel_y = std::min(std::max(target_vel_y, -max_speed_), max_speed_);
    vel_z = std::min(std::max(target_vel_z, -max_speed_), max_speed_);
    yaw_rate = 0.0f;  // 不偏航
    
    // 5. 累计距离
    total_distance_ += fabs(vel_x) * 0.05f;  // 20Hz控制频率
    
    // 6. 检查是否完成
    static int finish_counter = 0;
    if (!gate_detected_) 
    {
        finish_counter++;
        if (finish_counter > 20)  // 1秒内未检测到门
        {
            ROS_INFO("GateCrossing: 门消失，可能已通过，距离: %.2fm", total_distance_);
            return false;
        }
    }
    else 
    {
        finish_counter = 0;
        
        // 如果门在视野中很大且很近，也可能已穿过
        if (distance_ < close_distance_ && total_distance_ > 1.0f)
        {
            ROS_INFO("GateCrossing: 门很近且已飞行一定距离，可能已通过，距离: %.2fm", total_distance_);
            return false;
        }
    }
    
    // 7. 超时检查
    if (total_distance_ > target_distance_) 
    {
        ROS_INFO("GateCrossing: 已达到目标飞行距离: %.2fm", target_distance_);
        return false;
    }
    
    // 8. 输出调试信息
    static int info_counter = 0;
    if (info_counter++ % 10 == 0)  // 0.5秒输出一次
    {
        ROS_INFO("GateCrossing: 偏移=(%.2f,%.2f), 速度=(%.2f,%.2f,%.2f), 已飞=%.2fm, 距离估计=%.2fm", 
                offset_x_, offset_y_, vel_x, vel_y, vel_z, total_distance_, distance_);
        info_counter = 0;
    }
    
    return true;
}

void GateCrossing::start()
{
    active_ = true;
    total_distance_ = 0.0f;
    error_x_ = 0.0f;
    error_y_ = 0.0f;
    integral_x_ = 0.0f;
    integral_y_ = 0.0f;
    derivative_x_ = 0.0f;
    derivative_y_ = 0.0f;
    last_error_x_ = 0.0f;
    last_error_y_ = 0.0f;
    ROS_INFO("GateCrossing started");
}

void GateCrossing::stop()
{
    active_ = false;
    ROS_INFO("GateCrossing stopped");
}

void GateCrossing::reset()
{
    active_ = false;
    gate_detected_ = false;
    offset_x_ = 0.0f;
    offset_y_ = 0.0f;
    gate_width_ = 0.0f;
    distance_ = 0.0f;
    total_distance_ = 0.0f;
    error_x_ = 0.0f;
    error_y_ = 0.0f;
    integral_x_ = 0.0f;
    integral_y_ = 0.0f;
    derivative_x_ = 0.0f;
    derivative_y_ = 0.0f;
    last_error_x_ = 0.0f;
    last_error_y_ = 0.0f;
}

bool GateCrossing::isActive() const
{
    return active_;
}

bool GateCrossing::isGateDetected() const
{
    return gate_detected_;
}

float GateCrossing::getOffsetX() const
{
    return offset_x_;
}

float GateCrossing::getOffsetY() const
{
    return offset_y_;
}

float GateCrossing::getGateWidth() const
{
    return gate_width_;
}

float GateCrossing::getDistance() const
{
    return distance_;
}

float GateCrossing::getTotalDistance() const
{
    return total_distance_;
}

void GateCrossing::publishDebugImage()
{
    if (!has_image_ || current_image_.empty()) {
        return;
    }
    
    cv::Mat debug_image = current_image_.clone();
    int width = debug_image.cols;
    int height = debug_image.rows;
    
    // 绘制图像中心
    cv::Point center(width/2, height/2);
    cv::circle(debug_image, center, 5, cv::Scalar(0, 255, 0), 2);
    cv::line(debug_image, cv::Point(center.x-10, center.y), cv::Point(center.x+10, center.y), cv::Scalar(0, 255, 0), 1);
    cv::line(debug_image, cv::Point(center.x, center.y-10), cv::Point(center.x, center.y+10), cv::Scalar(0, 255, 0), 1);
    
    // 如果检测到门，绘制门的位置
    if (gate_detected_) 
    {
        // 转换为像素坐标
        float gate_center_x = (offset_x_ + 1.0f) * width / 2.0f;
        float gate_center_y = (offset_y_ + 1.0f) * height / 2.0f;
        
        cv::Point gate_center(gate_center_x, gate_center_y);
        cv::circle(debug_image, gate_center, 8, cv::Scalar(0, 0, 255), 2);
        
        // 绘制从中心到门的线
        cv::arrowedLine(debug_image, center, gate_center, cv::Scalar(255, 0, 0), 2);
        
        // 显示门信息
        std::string distance_text = "Distance: " + std::to_string(distance_) + "m";
        std::string offset_text = "Offset: (" + std::to_string(offset_x_) + ", " + std::to_string(offset_y_) + ")";
        std::string width_text = "Width: " + std::to_string(gate_width_) + "px";
        
        cv::putText(debug_image, distance_text, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        cv::putText(debug_image, offset_text, cv::Point(10, 60), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        cv::putText(debug_image, width_text, cv::Point(10, 90), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    } 
    else 
    {
        cv::putText(debug_image, "NO GATE DETECTED", cv::Point(width/2-100, height/2), 
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
    }
    
    // 转换为ROS图像消息并发布
    try {
        cv_bridge::CvImage debug_msg;
        debug_msg.encoding = sensor_msgs::image_encodings::BGR8;
        debug_msg.image = debug_image;
        debug_image_pub_.publish(debug_msg.toImageMsg());
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}
