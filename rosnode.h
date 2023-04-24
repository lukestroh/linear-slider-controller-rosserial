/*
    Header file for custom ethernet class
    Author: Luke Strohbehn
    Date: 4/17/2023

    Robots' ip addresses: 
        Joe's UR5 ip address: 169.254.174.50 
        Cindy's UR5 ip address: 169.254.177.232

    Host pc's ip addresses: 
        Desktop : 169.254.133.21 
        Laptop : 169.254.177.231
*/

#ifndef ROSNODE_H
#define ROSNODE_H

#include <stdint.h>
#include <Ethernet.h>
#include <Arduino.h>

#define ROSSERIAL_ARDUINO_TCP

#include <ros.h>
#include <std_msgs/Float32.h>

class ROSNode {
    private:
        // Connection parameters
        uint8_t MAC[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
        uint8_t IP[4] = { 169, 254, 235, 101 };
        uint8_t DNS[4] = { 169, 254, 235, 35 };
        uint8_t GATEWAY[4] = { 169, 254, 235, 35 };
        uint8_t SUBNET[4] = { 255, 255, 0, 0 };
        
        // Host server address, port
        IPAddress server;
        int local_port;

        // ROS node handler, chatter publisher
        ros::NodeHandle nh;
        

        // Private Methods
        void subscriber_callback();

    public:
        // Class declaration
        ROSNode(ros::NodeHandle& node_handle);

        // Class vars
        std_msgs::Float32 msg_f32;
        ros::Publisher publisher;
//        ros::Subscriber subscriber;        

        // Public Methods
        bool connect();

};

#endif // ROSNODE_H
