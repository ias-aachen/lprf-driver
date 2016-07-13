/* UDP client in the internet domain */
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ieee802154.h>

void error(const char *);
int main(int argc, char *argv[])
{
   int n;
   unsigned int length;
   struct sockaddr_ieee802154 server;
   struct sockaddr_ieee802154 from;
   struct hostent *hp;
   char buffer[256];
   
//   if (argc != 3) { printf("Usage: <server> <port>\n");
//                    exit(1);
//   }
   /* Create 802.15.4 socket */
   int sock = socket(AF_IEEE802154, SOCK_DGRAM, 0);
   if (sock < 0) 
      error("socket");

//   server.sin_family = AF_INET;
//   hp = gethostbyname(argv[1]);
//   if (hp==0) error("Unknown host");
//   bcopy((char *)hp->h_addr, (char *)&server.sin_addr, hp->h_length);
//   server.sin_port = htons(atoi(argv[2]));

   /* Fill in the from data fields */
   from.family           = AF_IEEE802154;
   from.addr.addr_type   = IEEE802154_ADDR_SHORT;
   from.addr.pan_id      = 0xdead;
   from.addr.short_addr  = 0xbeef;


   server.family          = AF_IEEE802154;
   server.addr.addr_type  = IEEE802154_ADDR_SHORT;
   server.addr.pan_id     = 0xdead;
   server.addr.short_addr = 0xbeee;




   length=sizeof(struct sockaddr_in);
   printf("Enter your message: ");
   bzero(buffer,256);
   fgets(buffer,255,stdin);
   n=sendto(sock, buffer, strlen(buffer), 0, (const struct sockaddr *)&server, length);
   if (n < 0) 
      error("Sendto");
//   n = recvfrom(sock, buffer, 256, 0, (struct sockaddr *)&from, &length);
//   if (n < 0) 
//      error("recvfrom");
   //write(1,"Got an ack: ",12);   
   //write(1,buffer,n);
//   printf("Got an ack: %s", buffer);
   close(sock);
   return 0;
}

void error(const char *msg)
{
    perror(msg);
    exit(0);
}
