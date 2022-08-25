#include <mutex>
#include <thread>
#include <chrono>
#include <condition_variable>
#include <boost/program_options.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>

#include "blaser_pcl/VoxelGridStitch.h"

std::string in_topic = "/blaser_pcl_topic";
std::string out_topic = "/blaser_pcl_stitched";
std::string srv_name = "/blaser_pcl/voxel_grid_stitch_srv";
std::string fixed_frame = "world";
float leaf_size = 0.0005;

static std::mutex gCollectingMtx;
static std::condition_variable gCollectingCV;

static std::atomic<bool> gCollecting(false);
static std::atomic<bool> gClearAccum(false);

namespace po = boost::program_options;

bool srv_callback(blaser_pcl::VoxelGridStitch::Request &req,
                  blaser_pcl::VoxelGridStitch::Response &res)
{
    if (req.start)
    {
        ROS_INFO("Started stitching");
        gCollecting = true;
    }
    else if (req.pause)
    {
        ROS_INFO("Paused stitching");
        gCollecting = false;
    }
    if (req.clear)
    {
        gClearAccum = true;
    }

    return true;
}

void parse_args(int argc, char **argv)
{
    po::options_description desc("Allowed options");
    desc.add_options()("help", "produce this message")("in_topic", po::value<std::string>(), "Input pointcloud topic (default blaser_pcl_topic)")("out_topic", po::value<std::string>(), "Output pointcloud topic (default blaser_pcl_stitched)")("srv_name", po::value<std::string>(), "Service name for interacting with stitcher")("fixed_frame", po::value<std::string>(), "Fixed frame to transform to")("leaf_size", po::value<float>(), "leaf size (meters) of the voxel grid filter");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(1);
    }

    if (vm.count("in_topic"))
    {
        in_topic = vm["in_topic"].as<std::string>();
        ROS_INFO("Listening to topic [%s].", in_topic.c_str());
    }

    if (vm.count("out_topic"))
    {
        out_topic = vm["out_topic"].as<std::string>();
        ROS_INFO("Broadcasting on topic [%s].", out_topic.c_str());
    }

    if (vm.count("srv_name"))
    {
        srv_name = vm["srv_name"].as<std::string>();
        ROS_INFO("Service name is [%s].", srv_name.c_str());
    }

    if (vm.count("fixed_frame"))
    {
        fixed_frame = vm["fixed_frame"].as<std::string>();
    }

    if (vm.count("leaf_size"))
    {
        leaf_size = vm["leaf_size"].as<float>();
    }
    ROS_INFO("Voxel grid filtering with leaf size %f!", leaf_size);
}

class VoxelFilterNode
{
    ros::NodeHandle nh;
    ros::Publisher filter_pub;
    ros::Subscriber pcl_sub;
    ros::ServiceServer service;

public:
    //pcl::VoxelGrid<pcl::PointXYZ> vg_filter;
    pcl::VoxelGrid<pcl::PCLPointCloud2> vg_filter;
    std::mutex cloud_accum_mtx;
    sensor_msgs::PointCloud2 cloud_accum;
    tf::TransformListener listener;
    bool has_data = false;

    VoxelFilterNode() //: cloud_accum(new PointCloud())
    {
        pcl_sub = nh.subscribe(in_topic.c_str(), 1, &VoxelFilterNode::pcl_callback, this);
        filter_pub = nh.advertise<pcl::PCLPointCloud2>(out_topic.c_str(), 1);

        vg_filter.setLeafSize(leaf_size, leaf_size, leaf_size);

        // Setup rosservice
        service = nh.advertiseService(srv_name.c_str(), srv_callback);
        ROS_INFO("ROS Service ready");
    }

    void pcl_callback(const sensor_msgs::PointCloud2 &input_pc)
    {
        tf::StampedTransform t;
        try
        {
            listener.waitForTransform(fixed_frame.c_str(), input_pc.header.frame_id, input_pc.header.stamp, ros::Duration(0.5));
            listener.lookupTransform(fixed_frame.c_str(), input_pc.header.frame_id,
                                     input_pc.header.stamp, t);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << std::endl;
            return;
        }

        sensor_msgs::PointCloud2 transformed;
        pcl_ros::transformPointCloud(fixed_frame.c_str(), t, input_pc, transformed);
        cloud_accum_mtx.lock();
        pcl::concatenatePointCloud(cloud_accum, transformed, cloud_accum);
        cloud_accum_mtx.unlock();

        has_data = true;
    }

    void publish_pc()
    {
        cloud_accum_mtx.lock();

        pcl::PCLPointCloud2::Ptr to_filter(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(cloud_accum, *to_filter);
        vg_filter.setInputCloud(to_filter);
        pcl::PCLPointCloud2::Ptr filtered(new pcl::PCLPointCloud2());
        vg_filter.filter(*filtered);
        ROS_INFO("filter has %dx%d points!", filtered->width, filtered->height);

        pcl_conversions::fromPCL(*filtered, cloud_accum);
        filter_pub.publish(cloud_accum);

        cloud_accum_mtx.unlock();
    }

    void clear_accum()
    {
        cloud_accum_mtx.lock();
        cloud_accum.data.clear();
        // Update height/width
        if (cloud_accum.height == 1)
        {
            cloud_accum.row_step = cloud_accum.width = 0;
        }
        else
        {
            if (cloud_accum.width == 1)
            {
                cloud_accum.height = 0;
            }
            else
            {
                cloud_accum.row_step = cloud_accum.width = cloud_accum.height = 0;
            }
        }
        cloud_accum_mtx.unlock();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "voxel_grid_filter_node");
    parse_args(argc, argv);
    VoxelFilterNode vfn;

    double rate = 10; //Hz
    ros::Time last_pub_t = ros::Time::now();
    while (ros::ok())
    {
        ros::spinOnce();
        while (gCollecting && ros::ok())
        {
            ros::spinOnce();

            if (vfn.has_data && (ros::Time::now() - last_pub_t).toSec() > 1.f / rate)
            {
                vfn.publish_pc();
                last_pub_t = ros::Time::now();
            }

            if (gClearAccum)
                break;

            std::this_thread::sleep_for(std::chrono::milliseconds(int(1000 / rate)));
        }

        if (gClearAccum)
        {
            vfn.clear_accum();
            vfn.publish_pc();
            gClearAccum = false;
            ROS_INFO("Cleared stitching");
        }

        // TODO: use conditional variable instead
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
