#include <iostream>

#include "y2018/vision_data.pb.h"
#include "y2018/common/constants.hpp"
#include "y2018/utils/netThread.hpp"

#include "aos/udp.h"
int main()
{
  // udp_client_server::udp_server server (TARGET_ADDR, UDP_PORT);
  int maxBufferSize = 1000000;
  int maxWaitSec = 1;
  char buf [1000000];
  y2018::vision::VisionData msg;
  char rawData[65507];

	::aos::events::RXUdpSocket recv(UDP_PORT);
  while (true)
  {
    // Receive data from non-blocking server
    // server.timed_recv(buf, maxBufferSize, maxWaitSec);
    int size = recv.Recv(rawData, 65507);
    // std::cout << "\nBuffer Size: " << strlen(buf) << std::endl;
    std::cout << "\nBuffer Size: " << size << std::endl;
    msg.ParseFromArray(rawData, size);
    std::cout << "\nImage timestamp: " << msg.image_timestamp() << std::endl;
    std::cout << "Send timestamp: " << msg.send_timestamp() << std::endl;
    std::cout << "Yaw: " << msg.yaw() << std::endl;
  }
  return 0;
}
