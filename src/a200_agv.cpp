#include <iostream>
#include <sstream>
#include "ros/ros.h"
#include "a200_agv.h"

#include "std_msgs/Empty.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"

#define DEBUG

#define LEFT 0
#define RIGHT 1

#define LINEAR 0
#define ANGULAR 1

#define DEG2RAD(x) (x * 0.01745329252) // *PI/180
#define RAD2DEG(x) (x * 57.2957795131) // *180/PI

#define constrain(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

void controlMotorTimer(const ros::TimerEvent &event);
void publishOdomTimer(const ros::TimerEvent &event);
void calOdomTimer(const ros::TimerEvent &event);
void cmdvelCB(const geometry_msgs::Twist &msg);
void resetCB(const std_msgs::Empty &msg);
void initOdom(void);

// std::string odom_header_frame_id;
// std::string odom_child_frame_id;

float wheel_vel[2] = {0.0, 0.0};
float goal_wheel_vel[2] = {0.0, 0.0};
float goal_velocity_from_cmd[2] = {0.0, 0.0};

float min_linear_velocity, max_linear_velocity;
float min_angular_velocity, max_angular_velocity;
float wheel_separation, wheel_radius;

ros::Subscriber cmdvel_sub;
ros::Subscriber reset_sub;
ros::Publisher odom_pub;

geometry_msgs::Pose2D pose, twist;
nav_msgs::Odometry odom;
geometry_msgs::TransformStamped odom_tf;

serial::Serial _comm;

ros::Time last_time;

bool pubOdomFrame;
uint8_t counter = 0;

int main(int argc, char **argv)
{
    std::cout << "Hello A200 AGV " << std::endl;
    ros::init(argc, argv, "a200_agv_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");

    std::string port;
    int baudrate;
    std::string odom_topic;
    std::string cmd_topic;

    nh_.param<std::string>("port", port, "/dev/ttySTM32");
    nh_.param<int>("baudrate", baudrate, 115200);
    nh_.param<float>("wheel_separation", wheel_separation, 0.5);
    nh_.param<float>("wheel_radius", wheel_radius, 0.1);

    nh_.param<float>("min_linear_velocity", min_linear_velocity, -1.0);
    nh_.param<float>("max_linear_velocity", max_linear_velocity, 1.0);
    nh_.param<float>("min_angular_velocity", min_angular_velocity, -5.0);
    nh_.param<float>("max_angular_velocity", max_angular_velocity, 5.0);

    nh_.param<bool>("publish_odom_frame", pubOdomFrame, true);
    nh_.param<std::string>("odom_topic", odom_topic, "odom");
    nh_.param<std::string>("cmd_topic", cmd_topic, "cmd_vel");

    nh_.param<std::string>("odom_frame_id", odom.header.frame_id, "odom");
    nh_.param<std::string>("base_frame_id", odom.child_frame_id, "base_footprint");

    cmdvel_sub = nh.subscribe("cmd_vel", 10, cmdvelCB);
    reset_sub = nh.subscribe("reset", 1, resetCB);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

    ros::Timer controlMotor = nh.createTimer(ros::Duration(0.15), controlMotorTimer);
    ros::Timer publishOdom = nh.createTimer(ros::Duration(0.1), publishOdomTimer);
    ros::Timer calOdom = nh.createTimer(ros::Duration(0.05), calOdomTimer);

    _comm.setPort(port);
    _comm.setBaudrate(baudrate);
    serial::Timeout to = serial::Timeout(50, 50, 0, 50, 0);
    _comm.setTimeout(to);

    initOdom();

    last_time = ros::Time::now();

#ifdef SERIAL
    while (ros::ok())
    {
        try
        {
            _comm.open();
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("Unable to open port.");
        }
        if (_comm.isOpen())
        {
            ROS_INFO_STREAM("Successfully connected to port.");
            try
            {
                ros::spin();
                // while (ros::ok())
                // {
                //     ros::spinOnce();
                // }
            }
            catch (const std::exception &e)
            {
                _comm.close();
                std::cerr << e.what() << '\n';
            }
        }
        else
        {
            ROS_WARN_STREAM("Could not connect to serial device "
                            << port << ". Trying again every 1 second.");
            ros::Duration(1.0).sleep();
            if (++counter >= 3)
            {
                ros::shutdown();
                return 0;
            }
        }
    }
#endif
    ros::spin();
    return 0;
}

void cmdvelCB(const geometry_msgs::Twist &msg)
{
    goal_velocity_from_cmd[LINEAR] = constrain(msg.linear.x, min_linear_velocity, max_linear_velocity);
    goal_velocity_from_cmd[ANGULAR] = constrain(msg.angular.z, min_angular_velocity, max_angular_velocity);
}

void resetCB(const std_msgs::Empty &msg)
{
    ROS_INFO("Reset Odometry");
    initOdom();
}

void initOdom(void)
{
    ROS_INFO_STREAM("initial odom");
    pose.x = 0.0;
    pose.y = 0.0;
    pose.theta = 0.0;

    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;
    odom.pose.pose.orientation.w = 0.0;

    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.angular.z = 0.0;
}

void controlMotorTimer(const ros::TimerEvent &event)
{
    ROS_INFO_STREAM("Write vel to motor");

    goal_wheel_vel[LEFT] = (goal_velocity_from_cmd[LINEAR] - wheel_separation / 2.0 * goal_velocity_from_cmd[ANGULAR]) / wheel_radius;
    goal_wheel_vel[RIGHT] = -(goal_velocity_from_cmd[LINEAR] + wheel_separation / 2.0 * goal_velocity_from_cmd[ANGULAR]) / wheel_radius;

#ifdef SERIAL
// writeVel(&_comm, goal_wheel_vel[LEFT], goal_wheel_vel[RIGHT]);
#endif
}

void publishOdomTimer(const ros::TimerEvent &event)
{
    ROS_INFO_STREAM("Publish odom");
    static tf::TransformBroadcaster tf_broadcaster;

    ros::Time current_time = ros::Time::now();

    odom.header.stamp = current_time;
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose.theta);

    odom.twist.twist.linear.x = twist.x;
    odom.twist.twist.angular.z = twist.theta;

    odom_pub.publish(odom);

    odom_tf.header.stamp = current_time;
    odom_tf.header = odom.header;
    odom_tf.child_frame_id = odom.child_frame_id;
    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;
    odom_tf.transform.rotation = odom.pose.pose.orientation;
    if (pubOdomFrame)
    {
        tf_broadcaster.sendTransform(odom_tf);
    }
}

void calOdomTimer(const ros::TimerEvent &event)
{
    ROS_INFO_STREAM("Calculate odom");
    ros::Time current_time = ros::Time::now();
    ros::Duration dt = current_time - last_time;
    last_time = current_time;

#ifdef SERIAL
// readVel(&_comm, wheel_vel);
// ROS_INFO_STREAM(wheel_vel[0] << ", " << wheel_vel[1]);
#endif

    float vL = wheel_vel[LEFT] * wheel_radius;
    float vR = wheel_vel[RIGHT] * wheel_radius;

    twist.x = (-vR + vL) / 2.0 + 0.01;
    twist.theta = (-vR - vL) / wheel_separation + 0.05;

    pose.x += (twist.x * cos(pose.theta)) * dt.toSec();
    pose.y += (twist.x * sin(pose.theta)) * dt.toSec();
    pose.theta += (twist.theta * dt.toSec());
}
