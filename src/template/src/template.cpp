#include <template.h>
#include<locale.h>
#include "gate_crossing.h"
using namespace std;

float target_x = 15.0f; 
float target_y = -0.5f; 

int mission_num = 0;           
float if_debug = 0.0f;         
float err_max = 0.2f;          

// 使用extern声明全局变量（在template.h中定义）
extern mavros_msgs::PositionTarget setpoint_raw;
extern mavros_msgs::State current_state;
extern nav_msgs::Odometry local_pos;

// 对象指针
CollisionAvoidance* avoidance = nullptr;
GateCrossing* gate_crossing = nullptr;

// 任务时间记录
ros::Time gate_crossing_start_time;
//gate_crossing
bool gate_crossing_active = false;
bool gate_crossing_complete = false;
float total_distance = 0.0f;
void print_param()
{
  std::cout << "=== 控制参数 ===" << std::endl;
  std::cout << "err_max: " << err_max << std::endl;
  std::cout << "ALTITUDE: " << ALTITUDE << std::endl;
  std::cout << "if_debug: " << if_debug << std::endl;
  if(if_debug == 1) cout << "自动offboard" << std::endl;
  else cout << "遥控器offboard" << std::endl;
}


int main(int argc, char **argv)
{
  // 防止中文输出乱码
  setlocale(LC_ALL, "");

  // 初始化ROS节点
  ros::init(argc, argv, "template");
  ros::NodeHandle nh;

  // 订阅mavros相关话题
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);
  // 添加避障相关的订阅和发布
   //ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/laser/scan", 1000, lidar_cb);
   
   // 创建对象
    avoidance = new CollisionAvoidance(nh);
    gate_crossing = new GateCrossing(nh);

    
  // 发布无人机多维控制话题
  ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

  // 创建服务客户端
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

  // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
  ros::Rate rate(20);

  // 参数读取

  nh.param<float>("err_max", err_max, 0);
  nh.param<float>("if_debug", if_debug, 0);
  print_param();
  
   
// 初始化模块
  if(!avoidance->initialize()) {
      ROS_ERROR("Failed to initialize collision avoidance module");
      delete avoidance;
      return -1;
  }
  if
 (!gate_crossing) {
    ROS_ERROR("Failed to create GateCrossing object!");
    return -1;
  }

  // 读取task参数
  nh.param<float>("target_x", target_x, 2.0f);
  nh.param<float>("target_y", target_y, 0.3f); 
  
  int choice = 0;
  std::cout << "1 to go on , else to quit" << std::endl;
  std::cin >> choice;
  if (choice != 1) return 0;
  ros::spinOnce();
  rate.sleep();
  
  // 等待连接到飞控
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }
  //设置无人机的期望位置
 
  setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
  setpoint_raw.coordinate_frame = 1;
  setpoint_raw.position.x = 0;
  setpoint_raw.position.y = 0;
  setpoint_raw.position.z = ALTITUDE;
  setpoint_raw.yaw = 0;

  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }
  std::cout<<"ok"<<std::endl;

  // 定义客户端变量，设置为offboard模式
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  // 定义客户端变量，请求无人机解锁
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  // 记录当前时间，并赋值给变量last_request
  ros::Time last_request = ros::Time::now();

  while (ros::ok())
  {
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
    {
      if(if_debug == 1)
      {
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
          ROS_INFO("Offboard enabled");
        }
      }
      else
      {
        ROS_INFO("Waiting for OFFBOARD mode");
      }
      last_request = ros::Time::now();
    }
    else
    {
      if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
      {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }
    // 当无人机到达起飞点高度后，悬停3秒后进入任务模式，提高视觉效果
    if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.2)
    {
      if (ros::Time::now() - last_request > ros::Duration(1.0))
      {
        mission_num = 1;
 	      last_request = ros::Time::now();
        break;
      }
    }

    mission_pos_cruise(0, 0, ALTITUDE, 0, err_max); 
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }
  
  while (ros::ok())
  {
    ROS_WARN("mission_num = %d", mission_num);
    
    switch (mission_num)
    {
      // mission1: 起飞
      case 1:
        if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
        {
          mission_num = 2;
          last_request = ros::Time::now();
        }
	    else if(ros::Time::now() - last_request >= ros::Duration(3.0))
        {
          mission_num = 2;
          last_request = ros::Time::now();
        }
        break;
        
        case 2:
      // 保持当前位置悬停
      mission_pos_cruise(0, 0, ALTITUDE, 0, err_max);
      
      // 检查是否已经悬停10秒
      if (ros::Time::now() - last_request > ros::Duration(10.0))
      {
        mission_num = 3;  // 悬停结束后进入下一个任务
        last_request = ros::Time::now();
        ROS_INFO("悬停10秒完成，开始下一个任务");
      }
      break;
       
// 避障前进（集成避障算法）
        case 3:

        {
            // 使用避障算法计算速度
              float vel_x, vel_y;
              if (avoidance->calculateAvoidanceVelocity(target_x, target_y, vel_x, vel_y))
              {
	    // 使用速度控制模式
	    setpoint_raw.type_mask = 0b000111000111;  // 速度控制模式
	    setpoint_raw.coordinate_frame = 1;
	    setpoint_raw.velocity.x = vel_x;
	    setpoint_raw.velocity.y = vel_y;
	    setpoint_raw.velocity.z = 0;  // 保持高度
	    setpoint_raw.yaw = 0;
	    
	    // 可选：打印避障信息
	    avoidance->printAvoidanceInfo();
}
            
            // 检查是否到达目标点
            float distance_to_target =  sqrt(pow(local_pos.pose.pose.position.x - (target_x + init_position_x_take_off), 2) + 
                                             pow(local_pos.pose.pose.position.y - (target_y + init_position_y_take_off), 2));

            if (distance_to_target < 0.7)
            {
                mission_num = 4;  // 避障完成后进入悬停状态
                last_request = ros::Time::now();
                ROS_INFO("避障任务完成，开始悬停10秒");
            }
            break;
        }
        
        // 避障完成后悬停5秒
        case 4:
            // 保持当前位置悬停
            mission_pos_cruise(target_x,target_y, ALTITUDE, 0, err_max);
            
            // 检查是否已经悬停5秒
            if (ros::Time::now() - last_request > ros::Duration(5.0))
            {
                mission_num = 5;  
                last_request = ros::Time::now();
                ROS_INFO("避障后悬停5秒完成，开始穿门");
            }
            break;
       
         
         // 穿门任务
case 5: 
{                
    static bool task5_initialized = false;
    static float task5_total_distance = 0.0f;
    static int task5_search_time = 0;
    static int task5_search_count = 0;
    
    if (!task5_initialized) 
    {                    
        // 初始化穿门任务   

        if (gate_crossing != nullptr) 
        {
            // 启动穿门模块
            gate_crossing->start();
            gate_crossing_active = true;
            gate_crossing_start_time = ros::Time::now();
            
            // 保存当前位置作为参考
            float current_x = local_pos.pose.pose.position.x;
            float current_y = local_pos.pose.pose.position.y;
            
            ROS_INFO("开始穿门任务，起始位置: (%.2f, %.2f, %.2f)", 
                    current_x, current_y, ALTITUDE);
            ROS_INFO("相机话题: /camera/image_raw");
        }
        else 
        {
            ROS_ERROR("GateCrossing对象未初始化!");
            mission_num = 6;  // 跳过穿门
            break;
        }
        
        task5_total_distance = 0.0f;
        task5_search_time = 0;
        task5_search_count = 0;
        task5_initialized = true;
    }                                
    
    // 检查门检测器是否有效
    if (gate_crossing == nullptr || !gate_crossing_active) 
    {
        ROS_ERROR("穿门模块无效，跳过");
        mission_num = 6;
        break;
    }
    
    // 超时保护（30秒）
    if ((ros::Time::now() - gate_crossing_start_time).toSec() > 10.0f) 
    {                    
        ROS_WARN("穿门超时（30秒），强制停止");
        ROS_INFO("飞行总距离: %.2fm", task5_total_distance);
        
        // 重置状态
        task5_initialized = false;
        gate_crossing_active = false;
        gate_crossing->stop();
        
        // 切换到位置控制模式
        setpoint_raw.type_mask = 0b110111111000;  // 位置控制模式
        setpoint_raw.coordinate_frame = 1;
        setpoint_raw.position.x = local_pos.pose.pose.position.x;
        setpoint_raw.position.y = local_pos.pose.pose.position.y;
        setpoint_raw.position.z = ALTITUDE;
        setpoint_raw.yaw = 0;
        
        // 进入下一任务
        mission_num = 6;  
        last_request = ros::Time::now();
        break;
    }
    
    // 更新穿门控制                
    float vel_x, vel_y, vel_z, yaw_rate;                
    bool continue_crossing = gate_crossing->update(vel_x, vel_y, vel_z, yaw_rate);
    
    if (continue_crossing) 
    {                    
        // 检查是否检测到门
        if (!gate_crossing->isGateDetected()) 
        {
            task5_search_count++;
            
            // 如果搜索超过5秒，尝试重新初始化
            if (task5_search_count > 100)  // 5秒（20Hz * 5）
            {
                ROS_WARN("长时间未检测到门，尝试重新搜索...");
                task5_search_count = 0;
                task5_search_time++;
                
                if (task5_search_time > 3)  // 总共尝试3次
                {
                    ROS_ERROR("多次尝试仍未检测到门，跳过穿门任务");
                    mission_num = 6;
                    break;
                }
            }
        }
        else 
        {
            task5_search_count = 0;  // 重置搜索计数器
            task5_search_time = 0;
        }
        
        // 设置穿门控制指令                    
        setpoint_raw.type_mask = 0b000111000111;  // 速度控制                    
        setpoint_raw.coordinate_frame = 1;                    
        setpoint_raw.velocity.x = vel_x;                    
        setpoint_raw.velocity.y = vel_y;                    
        setpoint_raw.velocity.z = vel_z;                    
        setpoint_raw.yaw_rate = yaw_rate;                                        
        
        // 记录飞行距离
        task5_total_distance += sqrt(vel_x*vel_x + vel_y*vel_y + vel_z*vel_z) * 0.05f;  // 20Hz控制频率                                        
        
        // 输出状态（每0.5秒一次）                    
        static int log_counter = 0;                    
        if (log_counter++ % 10 == 0) 
        {                        
            if (gate_crossing->isGateDetected()) 
            {
                ROS_INFO("门已检测: 偏移X=%.2f, 偏移Y=%.2f, 距离=%.2fm", 
                        gate_crossing->getOffsetX(), 
                        gate_crossing->getOffsetY(),
                        gate_crossing->getDistance());
            }
            else 
            {
                ROS_INFO("搜索门中... 旋转搜索");
            }
            
            ROS_INFO("速度: (%.2f, %.2f, %.2f) 累计距离: %.2fm", 
                    vel_x, vel_y, vel_z, task5_total_distance);                        
            log_counter = 0;                    
        }                
    } 
    else 
    {                    
        // 穿门完成                    
        ROS_INFO("=====================================");                    
        ROS_INFO("穿门完成！");                    
        ROS_INFO("总飞行距离: %.2fm", task5_total_distance);
        ROS_INFO("总用时: %.2f秒", (ros::Time::now() - gate_crossing_start_time).toSec());
        ROS_INFO("====================================="); 
        
        // 重置状态
        task5_initialized = false;
        gate_crossing_active = false;
        gate_crossing->stop();
        
        // 切换到位置控制模式
        setpoint_raw.type_mask = 0b110111111000;  // 位置控制模式
        setpoint_raw.coordinate_frame = 1;
        setpoint_raw.position.x = local_pos.pose.pose.position.x;
        setpoint_raw.position.y = local_pos.pose.pose.position.y;
        setpoint_raw.position.z = ALTITUDE;
        setpoint_raw.yaw = 0;
        
        // 进入下一任务
        mission_num = 6;  // 进入门后悬停                    
        last_request = ros::Time::now();
    }                                
    break;            
}
            
        case 6:  // 门后悬停
{                
    ROS_INFO_ONCE("任务6: 门后悬停");
    
    // 保持当前位置悬停
    mission_pos_cruise(local_pos.pose.pose.position.x,                                   
                       local_pos.pose.pose.position.y,                                   
                       ALTITUDE, 0, err_max);
    
    // 输出悬停信息
    static int hover_counter = 0;
    if (hover_counter++ % 20 == 0) 
    {  // 每秒一次
        ROS_INFO("门后悬停中...");
        hover_counter = 0;
    }
    
    // 检查是否已经悬停5秒
    if (ros::Time::now() - last_request > ros::Duration(5.0)) 
    {                    
        mission_num = 7;                
        last_request = ros::Time::now();                    
        
    }                
    break;            
}
      
     // 前进到x=16m - 位置控制
        case 7:
        {
            static bool case7_first = true;
            float current_y = local_pos.pose.pose.position.y;
            
            if (case7_first) {
                ROS_INFO("case7: 位置控制前进到x=16m");
                case7_first = false;
            }
            
            // 使用位置控制模式
            if (mission_pos_cruise(16.0f, current_y, ALTITUDE, 0, err_max))
            {
                ROS_INFO("到达x=16m，进入悬停2秒");
                mission_num = 8;
                last_request = ros::Time::now();
                case7_first = true;  // 重置状态
            }
            break;
        }
        
        // 在x=16m处悬停2秒 - 位置控制
        case 8:
        {
            static bool case8_first = true;
            float current_x = local_pos.pose.pose.position.x;
            float current_y = local_pos.pose.pose.position.y;
            
            if (case8_first) {
                ROS_INFO("case8: 位置控制悬停2秒");
                case8_first = false;
            }
            
            // 悬停时使用位置控制模式
            if (mission_pos_cruise(current_x, current_y, ALTITUDE, 0, err_max))
            {
                // 检查是否悬停了2秒
                if (ros::Time::now() - last_request > ros::Duration(2.0))
                {
                    ROS_INFO("悬停2秒完成，向右移动到(16, -3)");
                    mission_num = 9;
                    case8_first = true;  // 重置状态
                }
            }
            break;
        }
        
        // 向右移动到(16, -3) - 位置控制
        case 9:
        {
            static bool case9_first = true;
            
            if (case9_first) {
                ROS_INFO("case9: 位置控制向右移动到(16, -3)");
                case9_first = false;
            }
            
            // 使用位置控制模式
            if (mission_pos_cruise(16.0f, -3.0f, ALTITUDE, 0, err_max))
            {
                ROS_INFO("到达(16, -3)，开始前进到x=35m");
                mission_num = 10;
                case9_first = true;  // 重置状态
            }
            break;
        }
        
        // 前进到x=35m - 速度控制
        case 10:
        {
            static bool case10_first = true;
            float current_x = local_pos.pose.pose.position.x;
            
            if (case10_first) {
                ROS_INFO("case10: 速度控制前进到x=35m");
                case10_first = false;
            }
            
            // 计算误差
            float error_x = 35.0f - current_x;
            
            // 使用速度控制模式
            setpoint_raw.type_mask = 0b000111000111;  // 速度控制模式
            setpoint_raw.coordinate_frame = 1;
            
            // 设置速度，降低到0.5m/s
            float vel_x = 0.5f;
            if (error_x < 0) vel_x = -0.3f;  // 如果超过了目标，后退
            if (fabs(error_x) < 1.0f) vel_x = 0.4f;  // 接近目标时更慢
            if (fabs(error_x) < 0.5f) vel_x = 0.2f;  // 更接近时最慢
            
            setpoint_raw.velocity.x = vel_x;
            setpoint_raw.velocity.y = 0;  // 保持y坐标不变
            setpoint_raw.velocity.z = 0;  // 保持高度不变
            setpoint_raw.yaw = 0;
            
            // 检查是否到达目标
            if (fabs(error_x) < err_max)
            {
                ROS_INFO("到达x=35m，进入悬停2秒");
                mission_num = 11;
                last_request = ros::Time::now();
                case10_first = true;  // 重置状态
            }
            break;
        }
        
        // 在x=35m处悬停2秒 - 位置控制
        case 11:
        {
            static bool case11_first = true;
            float current_x = local_pos.pose.pose.position.x;
            float current_y = local_pos.pose.pose.position.y;
            
            if (case11_first) {
                ROS_INFO("case11: 位置控制悬停2秒");
                case11_first = false;
            }
            
            // 悬停时使用位置控制模式
            if (mission_pos_cruise(current_x, current_y, ALTITUDE, 0, err_max))
            {
                // 检查是否悬停了2秒
                if (ros::Time::now() - last_request > ros::Duration(2.0))
                {
                    ROS_INFO("悬停2秒完成，向左移动到(35, 3)");
                    mission_num = 12;
                    case11_first = true;  // 重置状态
                }
            }
            break;
        }
        
        // 向左移动到(35, -1) - 位置控制
          case 12:
    if (mission_pos_cruise(35, -1, ALTITUDE, 0, err_max))
    {
        ROS_INFO("到达(35, -1.0)，进入case13");
        mission_num = 13;
        last_request = ros::Time::now();
    }
    break;
    
                case 13:
      // 保持当前位置悬停
      mission_pos_cruise(35, -1, ALTITUDE, 0, err_max);
      
      // 检查是否已经悬停5秒
      if (ros::Time::now() - last_request > ros::Duration(5.0))
      {
        mission_num = 14;  // 悬停结束后进入下一个任务
        last_request = ros::Time::now();
        ROS_INFO("悬停5秒完成，开始下一个任务");
      }
      break;
       
       
      //降落
      case 14:
        if(precision_land())
        {
          mission_num = -1; // 任务结束
          last_request = ros::Time::now();
        }
        break;
    }
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
    
    if(mission_num == -1) 
    {
      exit(0);
    }
  }
  return 0;
}


