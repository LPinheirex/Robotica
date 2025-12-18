#include <ros/ros.h>
#include <boost/asio.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


// Variáveis globais para entrada do teclado e sincronização
std::string message;
std::mutex message_mutex; // Protege o acesso à variável 'message'
bool acesso = false;
double linear_velocity = 0.0;
double angular_velocity = 0.0;
unsigned long laserMicros;
bool firstData = true;
    // Variáveis de posição e velocidade
    double xx = 0.0, yy = 0.0, theta = 0.0;  // Posição inicial
    double dt = 0.1;  // Intervalo de tempo (10 Hz)
    double L = 0.1638;  // Comprimento base (distância entre rodas)

    double roll = 0.0, pitch = 0.0, yaw = 0.0; // Ângulos em radianos


// Função para gerenciar a entrada do teclado
void keyboardInputThread() {
    while (ros::ok()) {
        std::string input;
        std::cout << "Digite uma mensagem para enviar: ";
        std::getline(std::cin, input);

        if (input == "exit") {
            ros::shutdown(); // Para o ROS adequadamente
            break;
        }

        // Atualiza a mensagem de forma segura
        std::lock_guard<std::mutex> lock(message_mutex);
        message = input;
        acesso = true;
    }
}

void receiveMessages(boost::asio::ip::udp::socket& socket, 
                     ros::Publisher& odom_pub, 
                     ros::Publisher& imu_pub, 
                     ros::Publisher& scan_pub,
                     tf2_ros::TransformBroadcaster& odom_broadcaster) {    
    char recv_buffer[1024];
    boost::asio::ip::udp::endpoint sender_endpoint;

    geometry_msgs::Quaternion imu_orientation;

    while (ros::ok()) {
        boost::system::error_code error;
        size_t len = socket.receive_from(boost::asio::buffer(recv_buffer), sender_endpoint, 0, error);

        if (error && error != boost::asio::error::message_size) {
            std::cerr << "Erro ao receber: " << error.message() << std::endl;
            continue;
        }

        std::string received_message(recv_buffer, len);

        unsigned long micros;
        bool teste = false;

        int count = std::count(received_message.begin(), received_message.end(), '_');
        int parsed;

        if (count == 8){
            float vel, pos, xAcc, yAcc, zAcc, xRot, yRot, zRot;
            parsed = sscanf(received_message.c_str(), "%lu_%f_%f_%f_%f_%f_%f_%f_%f",
                                &micros, &vel, &pos, &xAcc, &yAcc, &zAcc, &xRot, &yRot, &zRot);
            if (parsed == 9) {
                teste = true;
             //   std::cout << vel << std::endl;            
                float pos_rad = pos * M_PI / 180.0;
                float rot = (vel / L) * tan(pos_rad);

                // Atualiza a posição
                xx += vel * dt * cos(theta);
                yy += vel * dt * sin(theta);
                theta += rot * dt;

                // Publica odometria
                nav_msgs::Odometry odom;
                odom.header.stamp = ros::Time::now();
                odom.header.frame_id = "odom";
                odom.child_frame_id = "base_link";

                odom.pose.pose.position.x = xx;
                odom.pose.pose.position.y = yy;
                odom.pose.pose.position.z = 0.0;

                tf2::Quaternion quat;
                quat.setRPY(0, 0, theta);
                geometry_msgs::Quaternion odom_quat = tf2::toMsg(quat);

                odom.pose.pose.orientation = odom_quat;

                odom.twist.twist.linear.x = vel;
                odom.twist.twist.linear.y = 0;
                odom.twist.twist.angular.z = rot;

                odom_pub.publish(odom);

                // Publica IMU
                sensor_msgs::Imu imu_msg;
                imu_msg.header.stamp = ros::Time::now();
                imu_msg.header.frame_id = "base_link";

                roll += xRot * dt;
                pitch += yRot * dt;
                yaw += zRot * dt;

                quat.setRPY(roll, pitch, yaw);
                imu_orientation = tf2::toMsg(quat);

                imu_msg.orientation = imu_orientation;
                imu_msg.angular_velocity.x = xRot;
                imu_msg.angular_velocity.y = yRot;
                imu_msg.angular_velocity.z = zRot;

                imu_msg.linear_acceleration.x = xAcc;
                imu_msg.linear_acceleration.y = yAcc;
                imu_msg.linear_acceleration.z = zAcc;

 

                imu_pub.publish(imu_msg);
            }
        }else{

            int points;
            int distOut[16]; // Vetor para armazenar os 16 valores de distância
            parsed = sscanf(received_message.c_str(),
                                "%lu_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d",
                                &micros, &points,
                                &distOut[0], &distOut[1], &distOut[2], &distOut[3],
                                &distOut[4], &distOut[5], &distOut[6], &distOut[7],
                                &distOut[8], &distOut[9], &distOut[10], &distOut[11],
                                &distOut[12], &distOut[13], &distOut[14], &distOut[15]);
            
            if (parsed == 18) {
                
                    std::cout << ((micros-laserMicros) / 1000000.0) << std::endl;            
//                    std::cout << received_message << std::endl;            
                
                if(firstData == true){
                    firstData = false;
                    laserMicros = micros;
                }else{
                    laserMicros = micros;
                    sensor_msgs::LaserScan scan_msg;

                    // Configuração básica da mensagem LaserScan
                    scan_msg.header.stamp = ros::Time::now();
                    scan_msg.header.frame_id = "laser_frame";
                    scan_msg.angle_min = 2.0 * M_PI * (1.0/16.0);
                    scan_msg.angle_max = 2.0 * M_PI * (17.0/16.0);
                    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / points;
//                    double tempo_tot = (micros-laserMicros) / 1000000.0;
  //                  double tempo_ind = ((micros-laserMicros) / 1000000.0) / points;
                    scan_msg.time_increment = 0.0625;
                    scan_msg.scan_time = 1.0;
                    scan_msg.range_min = 0.1;
                    scan_msg.range_max = 2;


                    // Preenche os dados de distância
                    scan_msg.ranges.resize(points);
                    for (int i = 0; i < points; ++i) {
                        scan_msg.ranges[i] = distOut[15 - i]/1000.0; // Considera 16 valores e os distribui
                    }



                    // Publica a mensagem
                    scan_pub.publish(scan_msg);
                }
            }
        }


    // Publicar transformação de odom -> base_link
    geometry_msgs::TransformStamped odom_to_base;
    odom_to_base.header.stamp = ros::Time::now();
    odom_to_base.header.frame_id = "odom";
    odom_to_base.child_frame_id = "base_link99";
    odom_to_base.transform.translation.x = xx;
    odom_to_base.transform.translation.y = yy;
    odom_to_base.transform.translation.z = 0.0;
    odom_to_base.transform.rotation = tf2::toMsg(tf2::Quaternion(0, 0, theta));

    odom_broadcaster.sendTransform(odom_to_base);
    

    // Publicar transformação de base_link -> laser_frame
    geometry_msgs::TransformStamped base_to_laser;
    base_to_laser.header.stamp = ros::Time::now();
    base_to_laser.header.frame_id = "base_link";
    base_to_laser.child_frame_id = "laser_frame";
    base_to_laser.transform.translation.x = 0.08;
    base_to_laser.transform.translation.y = 0.0;
    base_to_laser.transform.translation.z = 0.0;
    base_to_laser.transform.rotation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));

    odom_broadcaster.sendTransform(base_to_laser);

    // Publicar transformação de base_link -> imu_frame
    geometry_msgs::TransformStamped base_to_imu;
    base_to_imu.header.stamp = ros::Time::now();
    base_to_imu.header.frame_id = "base_link";
    base_to_imu.child_frame_id = "imu_frame";
    base_to_imu.transform.translation.x = 0.1;
    base_to_imu.transform.translation.y = 0.0;
    base_to_imu.transform.translation.z = 0.0;
    base_to_imu.transform.rotation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));

    odom_broadcaster.sendTransform(base_to_imu);

    }
}

// Atualizado callback de cmd_vel
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    acesso = true;
    message = "";

    if (msg->linear.x != 0 && msg->angular.z==0) {
        if (msg->linear.x > 0) {
            message += "Vel75";  // Exemplo: Velocidade positiva
//            message += "Acelerar0";  // Exemplo: Velocidade positiva
        } else {
            message += "Vel-75";  // Exemplo: Velocidade negativa
        }
    }

    if (msg->angular.z != 0 && msg->linear.x==0) {
        if (msg->angular.z > 0) {
            message += "Esquerda0";  // Exemplo: Girar para a esquerda
        } else {
            message += "Direita0";  // Exemplo: Girar para a direita
        }
    }

    if (msg->angular.z == 0 && msg->linear.x==0) {
            message += "Parar0";  // Exemplo: Girar para a esquerda
    }

  //  ROS_INFO("Mensagem gerada: %s", message.c_str());
}

void publishTransforms(tf2_ros::TransformBroadcaster& broadcaster) {

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "udp_receiver_sender");
    ros::NodeHandle nh;

    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 8080));
    std::cout << "Socket UDP vinculado à porta 8080 e aguardando mensagens..." << std::endl;

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 50);
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 50);
    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 10, cmdVelCallback);

    tf2_ros::TransformBroadcaster odom_broadcaster;

    // Thread para entrada de teclado
    std::thread keyboard_thread(keyboardInputThread);

    // Thread para recepção de mensagens UDP
    std::thread receiver_thread(receiveMessages, 
                                 std::ref(socket), 
                                 std::ref(odom_pub), 
                                 std::ref(imu_pub), 
                                 std::ref(scan_pub),
                                 std::ref(odom_broadcaster));

    boost::asio::ip::udp::socket send_socket(io_service);
    send_socket.open(boost::asio::ip::udp::v4());
    boost::asio::ip::udp::endpoint remote_endpoint(boost::asio::ip::address::from_string("192.168.4.22"), 8080);

    ros::Rate rate(10); // 10 Hz
    while (ros::ok()) {
        // Envia mensagem se disponível
        if (acesso) {
            std::lock_guard<std::mutex> lock(message_mutex);
            send_socket.send_to(boost::asio::buffer(message), remote_endpoint);
            acesso = false;
        }
        ros::spinOnce();
        rate.sleep();
    }

    socket.close();
    send_socket.close();
    receiver_thread.join();
    keyboard_thread.join();
    return 0;
}