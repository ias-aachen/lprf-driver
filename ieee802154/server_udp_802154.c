/* Creates a datagram server.  The port 
   number is passed as an argument.  This
   server runs forever */

#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <netdb.h>
#include <stdio.h>
#include <ieee802154.h>

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

int main(int argc, char *argv[])
{
   int length, n;
   socklen_t fromlen;
   struct sockaddr_ieee802154 server;
   struct sockaddr_ieee802154 from;
   char buf[1024];

//   if (argc < 2) {
//      fprintf(stderr, "ERROR, no port provided\n");
//      exit(0);
//   }
   
   int sock = socket(AF_IEEE802154, SOCK_DGRAM, 0);
   if (sock < 0) 
      error("Opening socket");
   /* Fill in server object */
   bzero(&server, sizeof(server));
   server.family          = AF_IEEE802154;
   server.addr.addr_type  = IEEE802154_ADDR_SHORT;
   server.addr.pan_id     = 0xdead;
   server.addr.short_addr = 0xbeef;

   if (bind(sock, (struct sockaddr *)&server, sizeof(server)) < 0) 
       error("binding");

   /* Fill in from object */
   bzero(&from, sizeof(from));
   from.family         = AF_IEEE802154;
   from.addr.addr_type  = IEEE802154_ADDR_SHORT;
   from.addr.pan_id     = 0xdead;
   from.addr.short_addr = 0xbeef;

   fromlen = sizeof(struct sockaddr_in);
   while (1) {
       n = recvfrom(sock, buf, 1024, 0, (struct sockaddr *)&from, &fromlen);
       if (n < 0) 
          error("recvfrom");
       printf("Received a datagram: %s", buf);
       n = sendto(sock, "Got your message\n", 17, 0, (struct sockaddr *)&from, fromlen);
       if (n  < 0) 
          error("sendto");
   }
   return 0;
 }

