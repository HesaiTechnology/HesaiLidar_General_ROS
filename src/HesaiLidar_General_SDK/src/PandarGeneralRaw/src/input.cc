/******************************************************************************
 * Copyright 2019 The Hesai Technology Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <iostream>
#include <sstream>
#include <time.h>
#include "../util.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

#include "src/input.h"
#include "log.h"

Input::Input(std::string deviceipaddr,uint16_t port, uint16_t gpsPort, std::string multicast_ip) {
  // LOG_D("port: %d, gpsPort: %d", port,gpsPort);
  socketForLidar = -1;
  const char* ip = deviceipaddr.c_str();
  const char* isIpV6 = strchr(ip, ':');
	// connect to Pandar UDP port
	if(isIpV6 == NULL)
	{
    socketForLidar = socket(PF_INET, SOCK_DGRAM, 0);
    if (socketForLidar == -1) {
      perror("socket");  // TODO(Philip.Pi): perror errno.
      return;
    }
    sockaddr_in myAddress;                     // my address information
    memset(&myAddress, 0, sizeof(myAddress));  // initialize to zeros
    myAddress.sin_family = AF_INET;            // host byte order
    myAddress.sin_port = htons(port);          // port in network byte order
    myAddress.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP

    if (bind(socketForLidar, reinterpret_cast<sockaddr *>(&myAddress),
            sizeof(sockaddr)) == -1) {
      perror("bind");  // TODO(Philip.Pi): perror errno
      return;
    }
    if(multicast_ip != ""){
      struct ip_mreq mreq;                      
      mreq.imr_multiaddr.s_addr=inet_addr(multicast_ip.c_str());
      mreq.imr_interface.s_addr = htonl(INADDR_ANY); 
      int ret = setsockopt(socketForLidar, IPPROTO_IP, IP_ADD_MEMBERSHIP, (const char *)&mreq, sizeof(mreq));
      if (ret < 0) {
        perror("Multicast IP error,set correct multicast ip address or keep it empty\n");
      } 
      else {
        printf("Recive data from multicast ip address %s\n", multicast_ip.c_str());
      }
    }
    if (fcntl(socketForLidar, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
      perror("non-block");
      return;
    }
    if (port == gpsPort) {
      socketNumber = 1;
      return;
    }
    // gps socket
    socketForGPS = -1;
    socketForGPS = socket(PF_INET, SOCK_DGRAM, 0);
    if (socketForGPS == -1) {
      perror("socket");  // TODO(Philip.Pi): perror errno.
      return;
    }

    int reuse = 1;
    int set_error = setsockopt(socketForGPS, SOL_SOCKET, SO_REUSEPORT, (const void *)&reuse, sizeof(int));
    if(set_error < 0) {
      perror("setsockopt");
    }

    sockaddr_in myAddressGPS;                        // my address information
    memset(&myAddressGPS, 0, sizeof(myAddressGPS));  // initialize to zeros
    myAddressGPS.sin_family = AF_INET;               // host byte order
    myAddressGPS.sin_port = htons(gpsPort);          // port in network byte order
    myAddressGPS.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

    if (bind(socketForGPS, reinterpret_cast<sockaddr *>(&myAddressGPS),
            sizeof(sockaddr)) == -1) {
      perror("bind");  // TODO(Philip.Pi): perror errno
      return;
    }
	}
	else
  {
    deviceipaddr = "";
    socketForLidar = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
    if (socketForLidar == -1) {
      perror("socket");  // TODO(Philip.Pi): perror errno.
      return;
    }
    sockaddr_in6 addr;
    memset(&addr, 0, sizeof(sockaddr_in6));
    // Prepare the sockaddr_in6 structure
    addr.sin6_family = AF_INET6;
    if (deviceipaddr.empty()) {
        addr.sin6_addr = in6addr_any;
    }
    else if (!inet_pton(AF_INET6, deviceipaddr.c_str(), &(addr.sin6_addr))) {
        std::cerr << "Invalid host address: " << deviceipaddr << std::endl;
        return ;
    }
    addr.sin6_port = htons(port);

    if (bind(socketForLidar, reinterpret_cast<sockaddr *>(&addr),
            sizeof(sockaddr_in6)) == -1) {
      perror("bind");  // TODO(Philip.Pi): perror errno
      return;
    }

    if (fcntl(socketForLidar, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
      perror("non-block");
      return;
    }
    if (port == gpsPort) {
      socketNumber = 1;
      return;
    }
    // gps socket
    socketForGPS = -1;

    socketForGPS = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
    if (socketForGPS == -1) {
      perror("socket");  // TODO(Philip.Pi): perror errno.
      return;
    }
    int reuse = 1;
    int set_error = setsockopt(socketForGPS, SOL_SOCKET, SO_REUSEPORT, (const void *)&reuse, sizeof(int));
    if(set_error < 0) {
      perror("setsockopt");
    }

    sockaddr_in6 addrGPS;
    memset(&addrGPS, 0, sizeof(sockaddr_in6));
    // Prepare the sockaddr_in6 structure
    addrGPS.sin6_family = AF_INET6;
    if (deviceipaddr.empty()) {
        addrGPS.sin6_addr = in6addr_any;
    }
    else if (!inet_pton(AF_INET6, deviceipaddr.c_str(), &(addrGPS.sin6_addr))) {
        std::cerr << "Invalid host address: " << deviceipaddr << std::endl;
        return ;
    }
    addrGPS.sin6_port = htons(gpsPort);
    if (bind(socketForGPS, reinterpret_cast<sockaddr *>(&addrGPS),
            sizeof(sockaddr_in6)) == -1) {
      perror("bind");  // TODO(Philip.Pi): perror errno
      return;
    }

    
    if(multicast_ip != ""){
      struct ipv6_mreq mreq6;
      if(!inet_pton(AF_INET6, multicast_ip.c_str(), &(mreq6.ipv6mr_multiaddr))) {
          std::cerr << "udp_server input multicast ip error! " << std::endl;
          return ;
      }
      mreq6.ipv6mr_interface = htonl(INADDR_ANY);
  
      int err = setsockopt(socketForLidar, IPPROTO_IPV6, IPV6_ADD_MEMBERSHIP, (const char *)&mreq6, sizeof(mreq6));
      setsockopt(socketForGPS, IPPROTO_IPV6, IPV6_ADD_MEMBERSHIP, (const char *)&mreq6, sizeof(mreq6));
      if (err)
      {
          std::cerr << "udp_server setsockopt IPPROTO_IPV6 IP_ADD_MEMBERSHIP failed: " << err << std::endl;
          return ;
      }
    }
	}

  if (fcntl(socketForGPS, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    perror("non-block");
    return;
  }
  socketNumber = 2;
}

Input::~Input(void) {
  if (socketForGPS > 0) close(socketForGPS);
  if (socketForLidar > 0) (void)close(socketForLidar);
}

// return : 0 - lidar
//          1 - gps
//          -1 - error
int Input::getPacket(PandarPacket *pkt) {
  struct pollfd fds[socketNumber];
  if (socketNumber == 2) {
    fds[0].fd = socketForGPS;
    fds[0].events = POLLIN;

    fds[1].fd = socketForLidar;
    fds[1].events = POLLIN;
  } else if (socketNumber == 1) {
    fds[0].fd = socketForLidar;
    fds[0].events = POLLIN;
  }
  static const int POLL_TIMEOUT = 1000;  // one second (in msec)

  sockaddr_in senderAddress;
  socklen_t senderAddressLen = sizeof(senderAddress);
  int retval = poll(fds, socketNumber, POLL_TIMEOUT);
  if (retval < 0) {  // poll() error?
    if (errno != EINTR) printf("poll() error: %s", strerror(errno));
    return -1;
  }
  if (retval == 0) {
    return -1;
  }
  if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
      (fds[0].revents & POLLNVAL)) {
    // device error?
    perror("poll() reports Pandar error");
    return -1;
  }

  senderAddressLen = sizeof(senderAddress);
  ssize_t nbytes;
  double time = getNowTimeSec();
  // printf("Real time: %lf\n",time);
  for (int i = 0; i != socketNumber; ++i) {
    if (fds[i].revents & POLLIN) {
      nbytes = recvfrom(fds[i].fd, &pkt->data[0], ETHERNET_MTU, 0,
                        reinterpret_cast<sockaddr *>(&senderAddress),
                        &senderAddressLen);
      break;
    }
  }

  if (nbytes < 0) {
    if (errno != EWOULDBLOCK) {
      perror("recvfail");
      return -1;
    }
  }
  pkt->size = nbytes;
  pkt->stamp = time;

  return 0;
}
