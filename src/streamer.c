/**
 * @author Cristian Axenie, cristian.axenie@tum.de
 * 
 * Simple tracking application using data streamed over TCP/IP 
 * from Axis IP Cam (Robot room / Holodeck), local camera or locally saved file.
 * The tracking algorithm will be used for mobile robot tracking in indoor operation.
 * 
 * Stream server definitions.
 */

#include "streamer.h"
#include "utils.h"
#include "tracking.h"

/**
 * Get client sockaddr for stream server 
 */
void *get_client_address(struct sockaddr *sa){
    /* check the type of socket */
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }
    /* if IPv6 */
    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

/*
 * Separate thread to accept socket connections
 * for sending the tracking data to a remote
 * machine
 */
void * remote_connections_handler(void *data){
    /* listen on sock_fd, new connection on new_fd */
    int sockfd, new_fd;  
    /* server info and specific structs */
    struct addrinfo hints, *servinfo, *p;
    /* client address info */
    struct sockaddr_storage client_addr; 
    socklen_t sin_size;
    /* serve socker options */
    int opt_flag = 1;
    /* return codes */
    int rv;

    /* init socket and connection options */
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE; // use my IP

    /* get server address info */
    if ((rv = getaddrinfo(NULL, port, &hints, &servinfo)) != 0) {
        printf("remote_connections_handler: %s\n", gai_strerror(rv));
        pthread_exit(NULL);
    }

    /* loop through all the results and bind to the first */
    for(p = servinfo; p != NULL; p = p->ai_next) {
        if ((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
            printf("remote_connections_handler: Cannot create socket\n");
            continue;
        }
	/* set socket options to aboid bind errors by reusing the address */
        if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt_flag, sizeof(int)) == -1) {
            printf("remote_connections_handler: Cannot set socket options\n");
            pthread_exit(NULL);
        }
	/* bind */
        if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
            close(sockfd);
            printf("remote_connections_handler: Cannot bind\n");
            continue;
        }
	/* get out if found one */
        break;
    }
    /* check binding status */
    if (p == NULL)  {
        printf("remote_connections_handler: Failed to bind\n");
        pthread_exit(NULL);
    }
    /* free non used space */
    freeaddrinfo(servinfo);  

    /* listen to clients */
    if (listen(sockfd, BACKLOG) == -1) {
        printf("remote_connections_handler: Cannot listen\n");
        pthread_exit(NULL);
    }

    /* main accept() loop for clients */
    while(1){ 
        /* prepare client data */
        sin_size = sizeof client_addr;
        if((new_fd = accept(sockfd, (struct sockaddr *)&client_addr, &sin_size))==-1){
	    printf("remote_connections_handler: Cannot accept\n");
            continue;
        }
	/* set up the client flag */
	client_on = 1;
	   /* main data streaming loop for the connected client */
           while(1){
		if(send_on==1){
		if (send(new_fd, obj->buffer, strlen(obj->buffer), 0) == -1){
                	printf("remote_connections_handler: Client disconnected \n");
			client_on = 0;
			send_on = 0;
			goto out;
		}
		send_on = 0;	
		}
	      }
    }
out: 
	close(new_fd);
	close(sockfd);
	printf("Exiting stream server ... \n");
	pthread_exit(NULL);
}
