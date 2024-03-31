#include <ros/ros.h>
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

  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

  ros::Subscriber odom_sub = nh.subscribe("/visual_slam/odom", 1000, OdomCallback);

  ros::Rate loop_rate(10);
  ros::Rate wait_rate(0.5);

  /**
  * A count of how many messages we have sent. This is used to create
  * a unique string for each message.
  */
  float min_x = -15;
  float max_x = 15;
  float min_y = -15;
  float max_y = 15;

  std::vector<geometry_msgs::PoseStamped> goals; 
  
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

  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();

    if(SPEED <= 0.00001)
    {
      if(count >= int(goals.size())) break;
      ROS_INFO("Stopped. Speed: %.2f. Move to next point.", SPEED);
      // Publish Goal Pose
      goal_pub.publish(goals[count]);
      ++count;
      wait_rate.sleep();
    }
    else
    {
      ROS_INFO("Moving at speed %.2f...", SPEED);
    }


  }
  return 0;
}
