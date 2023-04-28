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

#ifndef ETHTCP_H
#define ETHTCP_H

#include <stdint.h>
#include <Ethernet.h>
#include <Arduino.h>

class Eth {
    private:
        // Connection parameters
        uint8_t MAC[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
        uint8_t IP[4] = { 169, 254, 235, 101 };
        uint8_t DNS[4] = { 169, 254, 235, 35 };
        uint8_t GATEWAY[4] = { 169, 254, 235, 35 };
        uint8_t SUBNET[4] = { 255, 255, 0, 0 };

        

        // Server address, port, client
        IPAddress server;
        int local_port;
        EthernetClient client;

        // Private Methods
        void __init__();

    public:
        // Data buffer
        int numChars = 32;
        char receivedChars[32];
        bool newData = false;
        
        // Class declaration
        Eth();

        // Public Methods
        bool connect();
        void read_data();
        void send_data(int data);

};

#endif // ETHTCP_H
