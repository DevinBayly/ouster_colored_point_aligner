#define PCL_NO_PRECOMPILE // !! BEFORE ANY PCL INCLUDE!!
#include <rosbag/bag.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <chrono>
#include <map>
#include <tf2_msgs/TFMessage.h>
#include <tf2_msgs/TFMessage.h>
#include <nav_msgs/Path.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include "ros/ros.h"
#include <iostream>
#include "pcl/point_cloud.h"
#include <pcl/io/pcd_io.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>


#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>


struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint16_t ring;
    uint16_t ambient;
    uint32_t range;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint16_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)


void processLidarMeasurement(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    std::cout << "Received lidar measurement with seq ID " << cloud_msg->header.seq << std::endl;
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg(*cloud_msg, laserCloudIn);
    // getting sizes
    std::cout << "cloud size is" << laserCloudIn.points.size() << std::endl;
    // Publish the data
    // this->publisher.publish (cloud_msg);
    // pcl::io::savePCDFileBinary(std::to_string(cloud_msg->header.seq)+"_test_pcd.pcd", laserCloudIn);
}
int main(int argc, char **argv)
{
    // initialise the node

    // Specify the path to your ROS bag file
    std::string bag_file_path = "88-transform.bag";

    // Open the bag file for reading
    rosbag::Bag bag;
    try
    {
        bag.open(bag_file_path, rosbag::bagmode::Read);
    }
    catch (rosbag::BagIOException &ex)
    {
        ROS_ERROR("Failed to open bag file: %s", ex.what());
        return 1;
    }

    // Define a topic filter to read only TF2 messages
    std::vector<std::string> topics;
    topics.push_back("/aft_mapped_path");
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    std::map<double, geometry_msgs::Pose> map;
    // Loop through the messages in the bag
    for (rosbag::MessageInstance const &msg : view)
    {
        nav_msgs::Path::ConstPtr pth_msg = msg.instantiate<nav_msgs::Path>();
        if (pth_msg != nullptr)
        {
            // Process TF2 message here
            for (geometry_msgs::PoseStamped posestamped : pth_msg->poses)
            {
                geometry_msgs::Pose pose = posestamped.pose;
                // Access transform data
                // std::string child_frame_id = transform.child_frame_id;
                // std::string frame_id = transform.header.frame_id;
                // geometry_msgs::Transform transform_data = transform.transform;
                //// You can use the transform data as needed
                //// For example, you can access translation and rotation components
                // geometry_msgs::Vector3 translation = transform_data.translation;
                // geometry_msgs::Quaternion rotation = transform_data.rotation;
                //// Process the TF2 message data here
                //// ...

                // std::cout << "got message" << std::endl;
                // std::cout << posestamped.header.stamp << std::endl;
                // std::string frame_id = posestamped.header.frame_id;
                // std::cout << frame_id << std::endl;
                // std::cout << pose.position << std::endl;
                // std::cout << pose.orientation << std::endl;
                map[round(double(posestamped.header.stamp.sec) * 10.0) / 10.0] = pose;
            }
        }
    }
    // access the -pc bag and use the map
    {
        std::cout << "made it to the next part" <<std::endl;
        std::string bag_file_path = "88-pc.bag";

        // Open the bag file for reading
        rosbag::Bag bag_pc;
        try
        {
            bag_pc.open(bag_file_path, rosbag::bagmode::Read);
        }
        catch (rosbag::BagIOException &ex)
        {
            ROS_ERROR("Failed to open bag file: %s", ex.what());
            return 1;
        }

        int i =0;
        // Define a topic filter to read only TF2 messages
        std::vector<std::string> topics;
        topics.push_back("/ouster/points");
        rosbag::View view(bag_pc, rosbag::TopicQuery(topics));

        // Loop through the messages in the bag
        for (rosbag::MessageInstance const &msg : view)
        {
            sensor_msgs::PointCloud2::ConstPtr pc_msg = msg.instantiate<sensor_msgs::PointCloud2>();
            if (pc_msg != nullptr)
            {
                std::cout << "Received lidar measurement with seq ID " << pc_msg->header.seq << std::endl;
                std::cout << "Received lidar measurement with time " << pc_msg->header.stamp << std::endl;
                double rounded_time = round(double(pc_msg->header.stamp.sec) * 10.0) / 10.0;
                // look up the type that includes the intensity value
                pcl::PointCloud<Point> laserCloudIn;
                pcl::fromROSMsg(*pc_msg, laserCloudIn);
                // make an output cloud that
                pcl::PointCloud<Point> laserCloudOut;
                // get the transform information for using position and orientation to move the point cloud
                geometry_msgs::Pose p = map[rounded_time];
                Eigen::Quaterniond eigenQ = Eigen::Quaterniond(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
                Eigen::Affine3d transform = Eigen::Affine3d::Identity();
                Eigen::Vector3d pos(p.position.x,p.position.y,p.position.z);
                transform.translation() = pos;
                transform.rotate(eigenQ);

                pcl::transformPointCloud(laserCloudIn, laserCloudOut, transform);
                pcl::io::savePCDFileBinary(std::to_string(pc_msg->header.seq) + "_test_pcd.pcd", laserCloudOut);
            }
        }
        bag_pc.close();
    }

    // Close the bag file
    bag.close();

    // ros::init(argc, argv, "process_lidar");

    // std::cout << "Process_lidar node initialised" << std::endl;
    // // handle ROS communication events
    // ros::NodeHandle n;
    // ros::Subscriber sub = n.subscribe("/ouster/points", 1000, processLidarMeasurement);
    // ros::spin();

    return 0;
}