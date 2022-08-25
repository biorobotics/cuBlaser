#include <stdio.h> 
#include <stdlib.h> 
#include <errno.h> 
#include <string.h> 
#include <sys/types.h> 
#include <netinet/in.h> 
#include <sys/socket.h> 
#include <sys/wait.h> 

#define MYPORT 3490    /* the port users will be connecting to */

#define BACKLOG 10     /* how many pending connections queue will hold */

main()
{
  int sockfd, new_fd;  /* listen on sock_fd, new connection on new_fd */
  struct sockaddr_in my_addr;    /* my address information */
  struct sockaddr_in their_addr; /* connector's address information */
  int sin_size;

  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
    perror("socket");
    exit(1);
  }

  my_addr.sin_family = AF_INET;         /* host byte order */
  my_addr.sin_port = htons(MYPORT);     /* short, network byte order */
  my_addr.sin_addr.s_addr = INADDR_ANY; /* auto-fill with my IP */
  bzero(&(my_addr.sin_zero), 8);        /* zero the rest of the struct */

  if (bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) \
                                                                == -1) {
    perror("bind");
    exit(1);
  }

  printf("socket bind\n");
  
  if (listen(sockfd, BACKLOG) == -1) {
    perror("listen");
    exit(1);
  }

  printf("listener set up\n");

  char is_init = 0;

  while(1) {  /* main accept() loop */
        //printf("server: got connection from %s\n", \
    //                                   inet_ntoa(their_addr.sin_addr));
    if (is_init && send(new_fd, "Hello, world!\n", 14, 0) != -1)
      continue;
    else
    {
      printf("waiting for new connection\n");
      close(new_fd);
      //exit(0);
      //while(waitpid(-1,NULL,WNOHANG) > 0); /* clean up child processes */
      sin_size = sizeof(struct sockaddr_in);
      if ((new_fd = accept(sockfd, (struct sockaddr *)&their_addr, \
                                                         &sin_size)) == -1) {
        perror("accept");
        continue;
      }
      is_init = 1;
      printf("got connection\n");
    }
  }

}

