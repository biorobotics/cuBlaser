#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>

#define PORT 3490
#define MAXDATASIZE 100

ros::Publisher imu_pub;

struct imu_data_t
{
	int sec, usec;
	float ax, ay, az;
	float gx, gy, gz;
};

void procImuData(std::string& imu_data_str)
{
	double imu_arr[8];
    int sec;
	uint8_t data_cnt = 0;
	uint8_t start_idx = 0, end_idx;
	for(int i = 0; i < imu_data_str.size(); i++)
	{
		if (imu_data_str[i] == ',' || imu_data_str[i] == '\n' || imu_data_str[i] == '\0')
		{
			size_t len = i - start_idx;
			std::string data_sub_str = imu_data_str.substr(start_idx, len);
			start_idx = i + 1;
			imu_arr[data_cnt] = std::stof(data_sub_str);
			if (data_cnt == 0)
            {
                sec = std::stoi(data_sub_str);
                //std::cout << "sec: " << data_sub_str << std::endl;
            }
			data_cnt ++;
		}
	}

	sensor_msgs::Imu imu_data;
	imu_data.header.stamp.sec = sec;
	imu_data.header.stamp.nsec = int(imu_arr[1]) * 1000;
	imu_data.header.frame_id = "body";
	imu_data.linear_acceleration.x = imu_arr[2];
	imu_data.linear_acceleration.y = imu_arr[3];
	imu_data.linear_acceleration.z = imu_arr[4];
	imu_data.angular_velocity.x = imu_arr[5];
	imu_data.angular_velocity.y = imu_arr[6];
	imu_data.angular_velocity.z = imu_arr[7];

	imu_pub.publish(imu_data);
}

int main(int argc, char *argv[])
{
	// connect to tcp server
	int sockfd, numbytes;  
	char buf[MAXDATASIZE];
	struct hostent *he;
	struct sockaddr_in their_addr; /* connector's address information */

	if (argc != 2) {
        fprintf(stderr,"usage: client hostname\n");
        exit(1);
    }

	if ((he=gethostbyname(argv[1])) == NULL)
	{  /* get the host info */
        herror("gethostbyname");
        exit(1);
	}

	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("socket");
        exit(1);
	}

	their_addr.sin_family = AF_INET;      /* host byte order */
	their_addr.sin_port = htons(PORT);    /* short, network byte order */
	their_addr.sin_addr = *((struct in_addr *)he->h_addr);
	bzero(&(their_addr.sin_zero), 8);     /* zero the rest of the struct */

	if (connect(sockfd, (struct sockaddr *)&their_addr, \
		                              sizeof(struct sockaddr)) == -1) {
        perror("connect");
        exit(1);
	}

	// init ROS
	ros::init(argc, argv, "imu_cli");
	ros::NodeHandle n;
	imu_pub = n.advertise<sensor_msgs::Imu>("/imu", 100);

	while ((numbytes=recv(sockfd, buf, MAXDATASIZE, 0)) != -1) {
        buf[numbytes] = '\0';
	    printf("Received: %s\n",buf);
	    std::string imu_data_str(buf);
	    procImuData(imu_data_str);
    }
	close(sockfd);
    perror("recv");
    exit(1);
	
	return 0;
}

