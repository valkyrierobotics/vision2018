#include <vector>
#include <string>
#include <string.h>
#include <stdlib.h>
#include <cstdlib>
#include <iostream>

#include "include/utils/udpClientServer.hpp"

void sendPing (udp_client_server::udp_client& client)
{
    std::string msg = "@";
    printf("Sent Data: %s\n", msg.c_str());
	client.send(msg.c_str(), strlen(msg.c_str()));
}

int main (int argc, char** argv)
{
    std::string addr = "localhost";
    int port = 5810;
    if (argc > 1)
        addr = argv[1];
    if (argc > 2)
        port = std::atoi(argv[2]);
    udp_client_server::udp_client client (addr, port);
    sendPing(client);
    return 0;
}
