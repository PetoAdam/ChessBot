#include "chessbot/ZMQClient.hpp"
#include <string.h>

ZMQClient::ZMQClient(const std::string ipAddress){
    this->ipAddress=ipAddress;
    std::cout << "Connecting to address: " << ipAddress << std::endl;
    socket.connect(this->ipAddress);
}

void ZMQClient::send(const std::string data) {
    socket.send(zmq::buffer(data));
}

void ZMQClient::send(const char* data) {
    socket.send(zmq::buffer(std::string(data)));
}

std::string ZMQClient::recv() {
    zmq::message_t reply{};
    socket.recv(&reply);
    char* temp_string = static_cast<char*>(reply.data());
    return std::string(temp_string, reply.size());
}

ZMQClient::~ZMQClient(){
    socket.disconnect(ipAddress);
}