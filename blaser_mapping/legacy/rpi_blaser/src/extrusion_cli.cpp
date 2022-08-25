#include <stdio.h> 
#include <stdlib.h> 
#include <errno.h> 
#include <string.h> 
#include <netdb.h> 
#include <sys/types.h> 
#include <netinet/in.h>
#include <ros/ros.h>
#include <sys/socket.h>
#include <std_msgs/Bool.h>

#define PORT 3496    /* the port client will be connecting to */

#define MAXDATASIZE 100 /* max number of bytes we can get at once */

int sockfd;

void extrusion_cmd_cb(const std_msgs::Bool &data)
{
    char out_char;
    if (data.data == 0)
        out_char = '0';
    else
        out_char = '1';

    std::cout << "sending command" << std::endl;
    if (send(sockfd, &out_char, 1, 0) == -1)
    {
        std::cout << "send command failed! exit now ..." << std::endl;
        close(sockfd);
        exit(0);
    }
}

int main(int argc, char *argv[])
{
	int numbytes;
	char buf[MAXDATASIZE];
	struct hostent *he;
	struct sockaddr_in their_addr; /* connector's address information */

	if (argc != 2) {
        fprintf(stderr,"usage: client hostname\n");
        exit(1);
    }

	if ((he=gethostbyname(argv[1])) == NULL) {  /* get the host info */
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
    ros::Subscriber extrusion_cmd_sub = n.subscribe("/extrusion_cmd", 10, extrusion_cmd_cb);
    ros::spin();
	
	return 0;
}


