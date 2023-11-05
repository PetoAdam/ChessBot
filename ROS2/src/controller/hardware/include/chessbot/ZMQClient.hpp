#ifndef ZMQCLIENT_HPP
#define ZMQCLIENT_HPP

#include <string.h>
#include <zmq.hpp>
#include <iostream>

class ZMQClient {
private:
    std::string ipAddress;
    zmq::context_t context{1};
    zmq::socket_t socket{context, zmq::socket_type::req};
public:
    ZMQClient(const std::string ipAddress);
    ~ZMQClient();
    void send(const std::string data);
    void send(const char* data); 
    std::string recv();
};

#endif //ZMQCLIENT_HPP