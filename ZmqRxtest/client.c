#include <zmq.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

// to compile : 
// gcc client.c -o client -lzmq

typedef struct _sCplx
{
    float re;
    float im;

} TYPECPX;

int main (void)
{
    char message[2] ;
    TYPECPX *buffer ;
    int rc ;
    int len ;
    
    void *context = zmq_ctx_new ();
    void *subscriber = zmq_socket (context, ZMQ_SUB);

    printf ("Connecting to server…\n");
    zmq_connect (subscriber, "tcp://localhost:5563");

    // subscribe to "IQ" messages	
    zmq_setsockopt( subscriber, ZMQ_SUBSCRIBE, "IQ", 2);

    

    int request_nbr;
    for (;;) {
		// read header
		rc = zmq_recv( subscriber, message, 2, 0 );
		message[2] = 0 ;
		printf("header received rc=%d [%s] - ", rc, message );

		len = 0 ;
		rc = zmq_recv( subscriber, &len, sizeof(int), 0 );
		printf("size received rc=%d len=%d - ", rc, len );

		if( len > 0 ) {
			buffer = (TYPECPX *)malloc( len * sizeof(TYPECPX) );
			rc = zmq_recv( subscriber, buffer, len, 0);
			printf ("Received %d complex samples\n", rc/((int)sizeof(TYPECPX)) );
			free(buffer);
		} else {
			printf("Received Length == 0, last frame.\n");
		}
		
    }
    zmq_close (subscriber);
    zmq_ctx_destroy (context);
    return 0;
}

