#include "TCP/tcpSend.h"
using namespace ARRC;

TCP tcp("172.16.84.224");

double sent_data;
int main(){
    sent_data = 4;
    tcp.send(sent_data);
}
