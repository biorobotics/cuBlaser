/*
* point_cloud_server.cpp
* Listen to the point cloud topic and send data via a socket
* For AR applications
*
* Created by Luyuan Wang
*/

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>

#include <third_party/async-sockets/tcpserver.hpp> // 3rd party lib, under the MIT license

#include "std_msgs/String.h"

using namespace std;

static TCPServer tcpServer; // use static variables to prevent releasing tcpServer instance... which will cause seg fault
static TCPSocket *clientSocket;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

void pointCloudPCLCallback(PointCloud pc) {
    std::stringstream buffer;

    for (pcl::PointXYZRGB point : pc.points) {
        float x = point.x;
        float y = point.y;
        float z = point.z;

        std::uint32_t rgb = *reinterpret_cast<int*>(&point.rgb);
        std::uint8_t r = (rgb >> 16) & 0x0000ff;
        std::uint8_t g = (rgb >> 8)  & 0x0000ff;
        std::uint8_t b = (rgb)       & 0x0000ff;

        // TODO: a simple encoding method, should be replaced for better efficiency
        buffer  << x << " " << y << " " << z << " " << (float)r << " " << (float)g << " "  << (float)b << ";";
    }

    ROS_DEBUG("Sending point cloud...");
    if (clientSocket) {
        clientSocket->Send(buffer.str());
    }
}


void startServer() {
    // When a new client connected
    tcpServer.onNewConnection = [&](TCPSocket *newClient) {
        ROS_INFO_STREAM("New client: [" << newClient->remoteAddress() << ":" << newClient->remotePort() << "]");
        clientSocket = newClient;
    };

    // Bind the server to a port
    //TODO: read port number from a config file or command line arg
    tcpServer.Bind(8888, [](int errorCode, string errorMessage) {
        // BINDING FAILED:
        ROS_WARN_STREAM(errorCode << " : " << errorMessage);
    });

    // Start Listening the server
    tcpServer.Listen([](int errorCode, string errorMessage) {
        // LISTENING FAILED:
        ROS_WARN_STREAM(errorCode << " : " << errorMessage);
    });
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_server");

    ROS_INFO("Point Cloud Server starting...");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/slam_estimator/laser_pcd", 100, pointCloudPCLCallback);

    startServer();

    ros::Rate loop_rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}