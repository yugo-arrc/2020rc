#ifndef TCP_SEND_H
#define TCP_SEND_H

#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <cstring>

namespace ARRC{
    class TCP{
        public:
            TCP(std::string address);
            ~TCP(){
                close(sockfd);
            }
            bool send(float send_data);
        private:
            int sockfd;
            sockaddr_in addr;
            const std::string END = "x";
    };
};

#endif
