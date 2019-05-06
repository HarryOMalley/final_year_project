//
// Created by harry on 26/04/19.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>

//#define RAD2DEG(x) ((x)*180./M_PI)



sensor_msgs::LaserScan lastScan;
int lastRange=1;


class callbacks
{
public:
    float lastRange;
    ros::Publisher pub;
    sensor_msgs::LaserScan lastScan;
    void scanCallback(const sensor_msgs::PointCloudConstPtr& data);
    void tfminiCallback(const sensor_msgs::Range& data);
};

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

/*
 * Name: scanCallback
 *
 * This function is called whenever a new sensor_msgs:PointCloud message on the topic /slam_cloud is detected
 * It takes the last known value from the tfmini sensor and appends it to the z axis on the pointcloud to give it
 * a verticle component, then it publishes this on a new topic called /slam_cloud2
 */
void callbacks::scanCallback(const sensor_msgs::PointCloudConstPtr& data)
{
    ROS_INFO("Received point cloud: %s", data->header.frame_id.c_str());

    int count = data->points.size();

    // making the new pointcloud
    sensor_msgs::PointCloud heightAdjPC;

    float z = (rand() % 100000);
    z = lastRange;
    ROS_INFO("Setting height = %f", z);

    // appending a random height onto the pointcloud
    heightAdjPC.header = data->header;
    heightAdjPC.channels = data->channels;
    heightAdjPC.points = data->points;
    for(int i = 0; i<count; i++)
    {
        heightAdjPC.points[i].z = z;
    }

    heightAdjPC.header.stamp = ros::Time::now();
    ROS_INFO("Publishing %f", heightAdjPC.points[0].x);


    pub.publish(heightAdjPC);
}

/*
 * Name: tfminiCallback
 *
 * This function is called whenever a new sensor_msgs:Range message on the topic /mavros/rangefinder/rangefinder is detected
 * It takes the recorded range out and stores it in a global variable for use in the scanning callback
 */
void callbacks::tfminiCallback(const sensor_msgs::Range& data) {
    ROS_INFO("Received tfmini range: %f", data.range);

    float range = data.range;
    if (range < 2)
    {
        lastRange = range;
    }
}
int main(int argc, char **argv)
{
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "listener");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle nh;
    std::string scanTopic = nh.resolveName("/slam_cloud");
    std::string tfminiTopic = nh.resolveName("/mavros/rangefinder/rangefinder");
    uint32_t queue_size = 1;

    /**
     * The subscribe() call is how you tell ROS that you want to receive messages
     * on a given topic.  This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing.  Messages are passed to a callback function, here
     * called chatterCallback.  subscribe() returns a Subscriber object that you
     * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
     * object go out of scope, this callback will automatically be unsubscribed from
     * this topic.
     *
     * The second parameter to the subscribe() function is the size of the message
     * queue.  If messages are arriving faster than they are being processed, this
     * is the number of messages that will be buffered up before beginning to throw
     * away the oldest ones.s
     */

    callbacks callback;

    ros::Subscriber tfminiSub = nh.subscribe(tfminiTopic, queue_size, &callbacks::tfminiCallback, &callback);

    ros::Subscriber scanSub = nh.subscribe(scanTopic, queue_size, &callbacks::scanCallback, &callback);

    callback.pub = nh.advertise<sensor_msgs::PointCloud>("slam_cloud2", 1);

    while(callback.pub.getNumSubscribers() == 0)
    {
        ROS_INFO("No subscribers, please wait");
        //wait
    }
    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).  ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    ros::spin();

    return 0;
}

//void scanCallback2(const sensor_msgs::LaserScan::ConstPtr& scan)
//{
//    int count = scan->scan_time / scan->time_increment;
//    ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
//    ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
//    int length = scan->ranges.size(); // Will usually be 360
//}