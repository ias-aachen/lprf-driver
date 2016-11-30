#include <sys/types.h>
#include <sys/socket.h>
#include <ieee802154.h>
#include <stdio.h>
#include <errno.h>    /* errno  */

#include <string.h>   /* strerror() */
#include <strings.h>    /* bzero() */
#include <unistd.h>     /* sleep() */

int main()
{
        struct sockaddr_ieee802154 host_addr;
        struct sockaddr_ieee802154 dest_addr;

        /* Create a 802.15.4 socket */
        int sock = socket(AF_IEEE802154, SOCK_DGRAM, 0);
        if(sock < 0)
        {
                perror("socket");
                return -1;
        }

        /* Fill in the host_addr object */
        bzero(&host_addr, sizeof(host_addr));

        host_addr.family         = AF_IEEE802154;
        host_addr.addr.addr_type = IEEE802154_ADDR_SHORT;
        host_addr.addr.pan_id    = 0xdead;
        host_addr.addr.short_addr= 0xbeef;

	int a = bind(sock, (struct sockaddr *)&host_addr, sizeof(host_addr));
        if(a < 0)
        {
		printf("error integer: %d\n", a);
		int myerrno = errno;
		printf("myerrno=%s\n", strerror(myerrno));
                perror("bindingXX");
                return -1;
        }

        /* Fill in the dest_addr object */
        bzero(&dest_addr, sizeof(dest_addr));

        dest_addr.family    = AF_IEEE802154;
        dest_addr.addr.addr_type = IEEE802154_ADDR_SHORT;
        dest_addr.addr.pan_id    = 0xdead;
        dest_addr.addr.short_addr= 0xbeee;

	int b = 0;
        while(1)
        {
		printf("b:%d  ",b);
                if(sendto(sock, "Hello World", 12, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0)
                {
                        perror("sendto");
                        close(sock);
                        return -1;
                }
		printf("b:%d  ",b);
		b++;
                sleep(2);
        }

        close(sock);
        return 0;
}

