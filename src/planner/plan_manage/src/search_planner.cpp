#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

float speed[] = {100, 100, 100, 100};

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "search_node");
  ros::NodeHandle nh("~");

  // Publishers
  ros::Publisher goal_pub[] = {
    nh.advertise<geometry_msgs::PoseStamped>("/drone_0/waypoint_generator/move_base_simple/goal", 10),
    nh.advertise<geometry_msgs::PoseStamped>("/drone_1/waypoint_generator/move_base_simple/goal", 10),
    nh.advertise<geometry_msgs::PoseStamped>("/drone_2/waypoint_generator/move_base_simple/goal", 10),
    nh.advertise<geometry_msgs::PoseStamped>("/drone_3/waypoint_generator/move_base_simple/goal", 10)
  };

  // Odometry
  ros::Subscriber odom_sub[] = {
    nh.subscribe("/drone_0/visual_slam/odom", 1000, OdomCallback0),
    nh.subscribe("/drone_1/visual_slam/odom", 1000, OdomCallback1),
    nh.subscribe("/drone_2/visual_slam/odom", 1000, OdomCallback2),
    nh.subscribe("/drone_3/visual_slam/odom", 1000, OdomCallback3)
  };

  // Transforms
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // Publisher to send goal to ego_planner
  //ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

  // Subscriber to read odometry messages
  //ros::Subscriber odom_sub = nh.subscribe("/visual_slam/odom", 1000, OdomCallback);

  // List of goal pose
  std::vector<geometry_msgs::PoseStamped> goals; 


  // Add poses that go to each of the four corners
  float x = 21;
  float y = 21;

  geometry_msgs::PoseStamped goal;
  goal.pose.position.x = -x;
  goal.pose.position.y = -y;
  goal.pose.position.z = 1.0;
  goals.push_back(goal);

  goal.pose.position.x = -x;
  goal.pose.position.y = y;
  goal.pose.position.z = 1.0;
  goals.push_back(goal);

  goal.pose.position.x = x;
  goal.pose.position.y = -y;
  goal.pose.position.z = 1.0;
  goals.push_back(goal);

  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = 1.0;
  goals.push_back(goal);


  // Rates to define how often to check to go next step
  ros::Rate loop_rate(10);

  // Main Loop
  bool starting[] = {false, false, false, false};
  int count = 0;
  while (ros::ok())
  {
    // For each drone
    for(int i = 0; i < 4; i++)
    {
      // Get the pose of drone i
      geometry_msgs::TransformStamped tf;
      try
      {
        tf = tf_buffer.lookupTransform("world", "drone_" + std::to_string(i) + "_base", ros::Time(0));
        // ROS_INFO("Position: %.3f, %.3f, %.3f",
        //   tf.transform.translation.x,
        //   tf.transform.translation.y,
        //   tf.transform.translation.z
        // );
      }
      // Failed to get the pose of drone i
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        continue; 
      }

      // Command drone i
      if(speed[i] <= 0.00001 && !starting[i]) // check if stopped
      {
        if(count >= int(goals.size())) break; // if no more goals stop
        starting[i] = true;
        ROS_INFO("Moving drone %d to next point.", i);
        goal_pub[i].publish(goals[i]);
        ++count;
      }
      // Not stopped
      else
      {
        starting[i] = false; 
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
