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
#include <algorithm>
#include <vector>
#include "pcl/point_cloud.h"
#include <pcl/io/pcd_io.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>

struct EIGEN_ALIGN16 Point
{
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
    std::string param;
    ros::init(argc, argv, "node_name");
    ros::NodeHandle nh("~");
    nh.getParam("param", param);
    param = param.substr(1,param.length());
    ROS_INFO("Got parameter : %s", param.c_str());
    std::cout << param << std::endl;
    // Specify the path to your ROS bag file
    std::string bag_file_path = param+"-transform.bag";

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
    topics.push_back("/tf");
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    std::vector<geometry_msgs::Transform> map;
    // Loop through the messages in the bag
    for (rosbag::MessageInstance const &msg : view)
    {
        tf2_msgs::TFMessage::ConstPtr tf_message = msg.instantiate<tf2_msgs::TFMessage>();
        if (tf_message != nullptr) {
            // Process the TF message
            // You can access TF transform data from tf_message->transforms
            for (const geometry_msgs::TransformStamped& transform : tf_message->transforms) {
                // Access transform information like transform.child_frame_id, transform.header, etc.
                // For example:
                ROS_INFO("Frame ID: %s", transform.child_frame_id.c_str());
                map.push_back(transform.transform);
            }
        }
    }
    // access the -pc bag and use the map
    {
        std::cout << "made it to the next part" <<std::endl;
        std::string bag_file_path = param+"-pc.bag";

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
        // reverse the map so that we can pop the front items
        //std::reverse(map.begin(),map.end());
        // Loop through the messages in the bag
        std::ofstream ofile("transforms");
        ofile << "w,x,y,z,px,py,pz\n";
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
                geometry_msgs::Transform p = map[i];
                Eigen::Quaterniond eigenQ(p.rotation.w,p.rotation.x,p.rotation.y,p.rotation.z);
                Eigen::Vector3d pos( p.translation.x, p.translation.y, p.translation.z);
                ofile <<p.rotation.w << ",";
                ofile <<p.rotation.x<< ",";
                ofile <<p.rotation.y<< ",";
                ofile <<p.rotation.z<< ",";
                ofile <<p.translation.x<< ",";
                ofile <<p.translation.y<< ",";
                ofile <<p.translation.z<< "\n";
                Eigen::Affine3d transform = Eigen::Affine3d::Identity();
                transform.translation() = pos;
                transform.rotate(eigenQ);

                pcl::transformPointCloud(laserCloudIn, laserCloudOut, transform);
                pcl::io::savePCDFileBinary(std::to_string(pc_msg->header.seq) + "_test_pcd.pcd", laserCloudOut);
                i++;
            }
        }
        ofile.close();
        bag_pc.close();
    }

    // Close the bag file
    bag.close();


    return 0;
}
