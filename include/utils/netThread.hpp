#ifndef NET_THREAD_HPP
#define NET_THREAD_HPP

#include <iostream>
#include <cmath>
#include <cstring>
#include <unistd.h>

#include "udpClientServer.hpp"

const char START_SIGNAL = '@';
const char STOP_SIGNAL = '#';
const char GET_SIGNAL = 'G';
const char RESUME_SIGNAL = 'R';
const char PAUSE_SIGNAL = 'P';

const int LOOPS_PER_SEC = 1; 
const int MICROSEC_TO_SEC = 1000000;

char buff[50];

void sendData (udp_client_server::udp_client& client);
void receiveData (udp_client_server::udp_server& server);

#endif // NET_THREAD_HPP
