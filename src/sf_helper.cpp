#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>


class HelperSF
{
public:
    HelperSF() {
        sub_truth_odom = nh.subscribe<nav_msgs::Odometry> ("/truth_odom", 1, &HelperSF::callbackTruthOdom, this);
        pub_truth_path = nh.advertise<nav_msgs::Path>("/truth_path",1);

        sub = nh.subscribe<sensor_msgs::FluidPressure> ("/pressure_in", 1, &HelperSF::callback, this);
        pub = nh.advertise<sensor_msgs::FluidPressure>("/pressure_out",1);

        nh.param<std::string>("frame_id", frame_id, "odom");
    }

    ~HelperSF() {}

    void callback(const sensor_msgs::FluidPressure::ConstPtr& msg);

    void callbackTruthOdom(const nav_msgs::Odometry::ConstPtr& msg);

private:
    ros::NodeHandle nh;

    ros::Subscriber sub;
    ros::Publisher pub;

    // param
    std::string frame_id;

    // ground truth path
    ros::Publisher pub_truth_path;
    ros::Subscriber sub_truth_odom;
    nav_msgs::Path path_truth;
};

void HelperSF::callbackTruthOdom(const nav_msgs::Odometry::ConstPtr& msg){
    // ground truth path
    geometry_msgs::PoseStamped pose;

    pose.header.frame_id = frame_id;
    pose.header.stamp = msg->header.stamp;
    pose.pose.position.x = msg->pose.pose.position.x;
    pose.pose.position.y = msg->pose.pose.position.y;
    pose.pose.position.z = msg->pose.pose.position.z;

    path_truth.header.frame_id = frame_id;
    path_truth.header.stamp = msg->header.stamp;
    path_truth.poses.push_back(pose);

    pub_truth_path.publish(path_truth);
}

void HelperSF::callback(const sensor_msgs::FluidPressure::ConstPtr& msg){
  
    // for original stonefish simulator, it only generate Gauge pressure
    // manually add atmospheric pressure (101325 Pa)
    sensor_msgs::FluidPressure msg_out;
    msg_out.header = msg->header;
    msg_out.fluid_pressure = msg->fluid_pressure + 101325;
    pub.publish(msg_out);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Helper_SF_Node");

  HelperSF  example;

  ros::spin();
  return 0;
}