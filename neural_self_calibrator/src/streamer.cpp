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


/*
 * Separate thread to accept socket connections
 * for sending the tracking data to a remote
 * machine
 */
void *remote_connections_handler(void *arg)
{
    struct StaticData *data = (struct StaticData *)arg;

	/**
	 * Default behaviour, when client quits connection, is to exit streaming thread.
	 * Avoid this by ignoring the pipe interruption signal. 
	**/
    signal(SIGPIPE,SIG_IGN);


    int sockfd, newsockfd, portno;
    socklen_t clilen;
	
	/* server and client address structs */
    struct sockaddr_in serv_addr, cli_addr;

	/* create new TCP socket */
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
	/* set server socket to non blocking */
    fcntl(sockfd,F_SETFL,O_NONBLOCK);
	int val = 1;
	setsockopt(sockfd,SOL_SOCKET,SO_REUSEADDR,&val,sizeof(val));

	/* if server socket cannot be created, exit */ 
    if (sockfd < 0) error("ERROR opening socket");

    bzero((char *) &serv_addr, sizeof(serv_addr));
	/* assign port number */
    portno = data->PORT_NUM;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
	
	/* bind server address to socket */
    if (bind(sockfd, (struct sockaddr *) &serv_addr,
             sizeof(serv_addr)) < 0)
        error("ERROR on binding");
	
	/* mark server socket as passive, i.e. this socket listens for new incoming connections */
    listen(sockfd,5);

    clilen = sizeof(cli_addr);

    char buffer[256] = {'\0'};

    sprintf(buffer,"Connected\n");

    while (1)
    {
		/* if exit flag is set, quit streaming server */
        if(exit_state) break;

		/* accept new connection, i.e. create new socket for each client */
        newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
        if (newsockfd < 0)
        {
            if(errno==11) continue;
            error("ERROR on accept");
        }
		else
		{
			fprintf(stderr,"New client connected\n");
		}
		/* if client connected, enter data streaming loop*/
        while(1)
        {
			/* if exit flag is set, quit streaming server */
            if(exit_state) break;

			/* lock mutex to savely read global values */
            pthread_mutex_lock(&mutex);
			/* if not allowed to send, unlock mutex and continue */
            if(!marker_position->allowed_to_send)
            {
                pthread_mutex_unlock(&mutex);
                continue;
            }
            /* if allowed to send, send buffer content to client */
            if (send(newsockfd, marker_position->buffer, strlen(marker_position->buffer), 0) == -1)
            {
                fprintf(stderr,"Connection closed by client\nWaiting for new client to connect ... \n");
                marker_position->allowed_to_send = 0;
                pthread_mutex_unlock(&mutex);
                break;
            }

            marker_position->allowed_to_send = 0;
            pthread_mutex_unlock(&mutex);
        }

        close(newsockfd);
    }


    pthread_mutex_unlock(&mutex);
	close(newsockfd);
    close(sockfd);
    pthread_exit(NULL);
}


void error(const char *msg)
{
    perror(msg);
    exit(1);
}
