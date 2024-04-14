#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

float speed[] = {100, 100, 100, 100, 100, 100};

bool inRange(geometry_msgs::PoseStamped pose, geometry_msgs::TransformStamped tf, float range)
{
  float x = pose.pose.position.x - tf.transform.translation.x;
  float y = pose.pose.position.y - tf.transform.translation.y; 
  return ((x * x + y * y) <= range * range);
}

bool inRange(geometry_msgs::TransformStamped tf_0, geometry_msgs::TransformStamped tf_1, float range)
{
  float x = tf_0.transform.translation.x - tf_1.transform.translation.x;
  float y = tf_0.transform.translation.y - tf_1.transform.translation.y; 
  return ((x * x + y * y) <= range * range);
}

void OdomCallback(const nav_msgs::Odometry msg, int i)
{
  float vel_x = msg.twist.twist.linear.x;
  float vel_y = msg.twist.twist.linear.y;
  float vel_z = msg.twist.twist.linear.z;
  
  speed[i] = std::sqrt(vel_x * vel_x + vel_y * vel_y + vel_z * vel_z);
}

void OdomCallback0(const nav_msgs::Odometry msg) { OdomCallback(msg, 0); }
void OdomCallback1(const nav_msgs::Odometry msg) { OdomCallback(msg, 1); }
void OdomCallback2(const nav_msgs::Odometry msg) { OdomCallback(msg, 2); }
void OdomCallback3(const nav_msgs::Odometry msg) { OdomCallback(msg, 3); }
void OdomCallback4(const nav_msgs::Odometry msg) { OdomCallback(msg, 4); }
void OdomCallback5(const nav_msgs::Odometry msg) { OdomCallback(msg, 5); }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "search_node");
  ros::NodeHandle nh("~");

  // Publishers
  ros::Publisher goal_pub[] = {
    nh.advertise<geometry_msgs::PoseStamped>("/drone_0/waypoint_generator/move_base_simple/goal", 10),
    nh.advertise<geometry_msgs::PoseStamped>("/drone_1/waypoint_generator/move_base_simple/goal", 10),
    nh.advertise<geometry_msgs::PoseStamped>("/drone_2/waypoint_generator/move_base_simple/goal", 10),
    nh.advertise<geometry_msgs::PoseStamped>("/drone_3/waypoint_generator/move_base_simple/goal", 10),
    nh.advertise<geometry_msgs::PoseStamped>("/drone_4/waypoint_generator/move_base_simple/goal", 10),
    nh.advertise<geometry_msgs::PoseStamped>("/drone_5/waypoint_generator/move_base_simple/goal", 10)
  };

  // Odometry
  ros::Subscriber odom_sub[] = {
    nh.subscribe("/drone_0/visual_slam/odom", 1000, OdomCallback0),
    nh.subscribe("/drone_1/visual_slam/odom", 1000, OdomCallback1),
    nh.subscribe("/drone_2/visual_slam/odom", 1000, OdomCallback2),
    nh.subscribe("/drone_3/visual_slam/odom", 1000, OdomCallback3),
    nh.subscribe("/drone_4/visual_slam/odom", 1000, OdomCallback4),
    nh.subscribe("/drone_5/visual_slam/odom", 1000, OdomCallback5)
  };

  // Transforms
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  geometry_msgs::TransformStamped poses[6];

  // List of goal pose
  bool at_target[6];
  geometry_msgs::PoseStamped goal;
  geometry_msgs::PoseStamped goals[6]; 

  // Search
  bool move_lateral[4] = {true, true, true, true};

  // Circle
  int target_pose_index[] = {0, 2};
  geometry_msgs::PoseStamped circle[4];

  goal.pose.position.x = 5.0;
  goal.pose.position.y = 5.0;
  goal.pose.position.z = 1.0;
  circle[0] = goal;
  goal.pose.position.x = -5.0;
  goal.pose.position.y = 5.0;
  goal.pose.position.z = 1.0;
  circle[1] = goal;
  goal.pose.position.x = -5.0;
  goal.pose.position.y = -5.0;
  goal.pose.position.z = 1.0;
  circle[2] = goal;
  goal.pose.position.x = 5.0;
  goal.pose.position.y = -5.0;
  goal.pose.position.z = 1.0;
  circle[3] = goal;
  
  // Rates to define how often to check to go next step
  ros::Rate loop_rate(1);

  bool first_success = true;
  // Main Loop
  while (ros::ok())
  {
    bool in_pursuit[4] = {false, false, false, false};

    // Get Position of Each Drone
    bool tf_success = true; 
    for(int i = 0; i < 6; i++)
    {
      try
      {
        poses[i] = tf_buffer.lookupTransform(
            "world", 
            "drone_" + std::to_string(i) + "_base", 
            ros::Time(0));
      }
      catch(tf2::TransformException &ex)
      {
        //ROS_WARN("%s", ex.what()); 
        tf_success = false;
      }
    }
    if(!tf_success) {continue;}
    else
    {
      if(first_success)
      {
        for(int i = 0; i < 6; i++)
        {
          goal.pose.position.x = poses[i].transform.translation.x;
          goal.pose.position.y = poses[i].transform.translation.y;
          goal.pose.position.z = poses[i].transform.translation.z;
          goals[i] = goal;
        }
        first_success = false;
      }
    }

    // Check if Drones are close to their Goals
    for(int i = 0; i < 6; i++)
    {
      at_target[i] = inRange(goals[i], poses[i], 0.5);
    }

    // Check if Pursuers are Close to Targets
    for(int i = 0; i < 4; i++)
    {
      for(int j = 4; j < 5; j++)
      {
        if(inRange(poses[i], poses[j], 5))
        {
          goal.pose.position.x = poses[j].transform.translation.x;
          goal.pose.position.y = poses[j].transform.translation.y;
          goal.pose.position.z = poses[j].transform.translation.z;
          goals[i] = goal;
          in_pursuit[i] = true;
        }
      }
    }

    // Set Goals for Pursuers
    for(int i = 0; i < 4; i++)
    {
      if(at_target[i] && !in_pursuit[i])
      {
        if(move_lateral[i])
        {
          goal.pose.position.x = -poses[i].transform.translation.x;
          goal.pose.position.y = poses[i].transform.translation.y;
          goal.pose.position.z = poses[i].transform.translation.z;
          move_lateral[i] = !move_lateral[i];
          goals[i] = goal;
        }
        else
        {
          goal.pose.position.x = poses[i].transform.translation.x;
          goal.pose.position.y = poses[i].transform.translation.y + 2.5;
          goal.pose.position.z = poses[i].transform.translation.z;
          move_lateral[i] = !move_lateral[i];
          goals[i] = goal;
        }
      }
    }

    // Set Goals for Targets 
    for(int i = 4; i < 6; i++)
    {
      if(at_target[i])
      {
        target_pose_index[i - 4] += 1;
        if(target_pose_index[i - 4] == 4)
        {
          target_pose_index[i - 4] = 0;
        }
        goals[i] = circle[target_pose_index[i - 4]]; 
      }
    }

    // Move Drones
    for(int i = 0; i < 6; i++)
    {
      if(at_target[i] || in_pursuit[i])
      {
        goal_pub[i].publish(goals[i]);
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
