// https://en.wikipedia.org/wiki/SocketCAN
// https://lnguin.wordpress.com/tag/socketcan-example/
// https://stackoverflow.com/questions/21135392/socketcan-continuous-reading-and-writing

#include <stdio.h>
//#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
//#include <sys/types.h>
//#include <sys/socket.h>
#include <sys/ioctl.h>
//#include <fcntl.h>

#include <linux/can.h>	
//#include <linux/can/raw.h>

static int soc;

int can_init(const char *port) {
	static struct sockaddr_can addr;
	static struct ifreq ifr;

	if((soc = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Error while opening socket");
		return(-1);
	}

	strcpy(ifr.ifr_name, port);
	
	if(ioctl(soc, SIOCGIFINDEX, &ifr) < 0) {
		perror("Error in ioctl()");
		return(-1);
	} else {
		//printf("%s at index %d\n", port, ifr.ifr_ifindex);
	}
	
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if(bind(soc, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Error in socket bind");
		return(-1);
	}
	
	//printf("Opened: %s\r\n", port);
	
	return 0;
}

int can_send(struct can_frame *frame) {
    int retval;
	retval = write(soc, frame, sizeof(struct can_frame));
    if (retval != sizeof(struct can_frame))
    {
        return(-1);
    }
    else
    {
        return(0);
    }
}

int can_read(struct can_frame *frame) {
    int retval;
	retval = read(soc, frame, sizeof(struct can_frame));
	if (retval != sizeof(struct can_frame))
    {
        return(-1);
    }
    else
    {
        return(0);
    }
}
