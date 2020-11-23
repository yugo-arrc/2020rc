#include "tcpSend.h"

namespace ARRC{
    TCP::TCP(std::string address){
        if( (sockfd = socket( AF_INET, SOCK_STREAM, 0) ) < 0 ) {
            std::cerr << "socket" << std::endl;
        }
        addr.sin_family = AF_INET;
        addr.sin_port = htons( 54321 );
        addr.sin_addr.s_addr = inet_addr(address.c_str());
        connect( sockfd, (struct sockaddr *)&addr, sizeof( struct sockaddr_in ));
    }

    bool TCP::send(float send_data){
        char buf[255];
        std::memset(buf, 0x00, sizeof(buf));
        std::string msg = std::to_string(send_data)+END;
        if(::send(sockfd, msg.c_str(), msg.size(), 0)<0){
            std::cerr << "failed to send" << std::endl;
            return false;
        }else{
            recv(sockfd, buf, sizeof(buf), 0);
            std::cout << buf << std::endl;
        }
        return true;
    }


}