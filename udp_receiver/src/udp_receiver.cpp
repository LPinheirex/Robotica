#include <ros/ros.h>
#include <boost/asio.hpp>
#include <iostream>
#include <string>

int main(int argc, char **argv) {
    // Inicializa o nó ROS
    ros::init(argc, argv, "udp_receiver");
    ros::NodeHandle nh;

    // Inicializa o serviço de IO do Boost.Asio
    boost::asio::io_service io_service;

    // Cria o socket UDP e vincula ao IP e porta específicos
    boost::asio::ip::udp::socket socket(io_service);

    // Usa INADDR_ANY para vincular o socket a todas as interfaces, usando `0.0.0.0`
    boost::asio::ip::udp::endpoint endpoint(boost::asio::ip::udp::v4(), 8080);
    socket.open(endpoint.protocol());
    socket.bind(endpoint);

    std::cout << "Socket UDP vinculado à porta " << endpoint.port() << " e aguardando mensagens..." << std::endl;

    // Buffer para armazenar os dados recebidos
    char recv_buffer[1024];
    boost::asio::ip::udp::endpoint sender_endpoint;

    boost::asio::ip::address ip_addresss = boost::asio::ip::address::from_string("192.168.4.22");
        boost::asio::ip::udp::endpoint endpointt(ip_addresss, 8080);

    while (ros::ok()) {
        // Recebe a mensagem UDP
        boost::system::error_code error;
        size_t len = socket.receive_from(boost::asio::buffer(recv_buffer), endpointt, 0, error);

        std::cout << "Endereco: " << endpointt << std::endl;
        if (error && error != boost::asio::error::message_size) {
            std::cerr << "Erro ao receber: " << error.message() << std::endl;
            continue;
        }

        // Converte o buffer recebido para uma string
        std::string received_message(recv_buffer, len);
        std::cout << "Mensagem recebida: " << received_message << std::endl;
    }

    return 0;
}
