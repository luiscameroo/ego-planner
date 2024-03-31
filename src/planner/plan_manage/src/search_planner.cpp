#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

float SPEED = 100;

void OdomCallback(const nav_msgs::Odometry msg)
{
  float vel_x = msg.twist.twist.linear.x;
  float vel_y = msg.twist.twist.linear.y;
  float vel_z = msg.twist.twist.linear.z;
  
  SPEED = std::sqrt(vel_x * vel_x + vel_y * vel_y + vel_z * vel_z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "search_node");
  ros::NodeHandle nh("~");

  // Publisher to send goal to ego_planner
  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

  // Subscriber to read odometry messages
  ros::Subscriber odom_sub = nh.subscribe("/visual_slam/odom", 1000, OdomCallback);

  // Subsciber to Listen to transform
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // List of goal pose
  std::vector<geometry_msgs::PoseStamped> goals; 


  // Add poses that go to each of the four corners
  float min_x = -15;
  float max_x = 15;
  float min_y = -15;
  float max_y = 15;

  geometry_msgs::PoseStamped goal;
  goal.pose.position.x = min_x;
  goal.pose.position.y = min_y;
  goal.pose.position.z = 1.0;
  goals.push_back(goal);

  goal.pose.position.x = min_x;
  goal.pose.position.y = max_y;
  goal.pose.position.z = 1.0;
  goals.push_back(goal);

  goal.pose.position.x = max_x;
  goal.pose.position.y = max_y;
  goal.pose.position.z = 1.0;
  goals.push_back(goal);

  goal.pose.position.x = max_x;
  goal.pose.position.y = min_y;
  goal.pose.position.z = 1.0;
  goals.push_back(goal);

  goal.pose.position.x = min_x;
  goal.pose.position.y = min_y;
  goal.pose.position.z = 1.0;
  goals.push_back(goal);

  // Rates to define how often to check to go next step
  ros::Rate loop_rate(10);
  ros::Rate wait_rate(0.1);

  // Main Loop
  int count = 0;
  while (ros::ok())
  {
    // Get the current pose of the drone
    geometry_msgs::TransformStamped tf;
    try
    {
      tf = tf_buffer.lookupTransform("world", "base", ros::Time(0));
      ROS_INFO("Position: %.3f, %.3f, %.3f",
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z
      );
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue; 
    }

    if(SPEED <= 0.00001)
    {
      if(count >= int(goals.size())) break;
      ROS_INFO("Stopped. Speed: %.3f. Move to next point.", SPEED);
      // Publish Goal Pose
      goal_pub.publish(goals[count]);
      ++count;
      // Wait for motion to commence
      wait_rate.sleep();
    }
    else
    {
      ROS_INFO("Moving at speed %.3f...", SPEED);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
