/*
    Source file for custom Ethernet class
    Author: Luke Strohbehn
    Date: 4/17/2023
*/

#include "ROSNode.h"

ROSNode::ROSNode(ros::NodeHandle& node_handle) {
    /* Set class variables */
    server = IPAddress(169, 254, 235, 35);
    local_port = 10002;
    
    nh = node_handle;
    publisher = nh.advertise<std_msgs::Float32>("slider_pos", 10, true); // queue_size: Maximum number of outgoing messages to be queued for Subscribers
    //    subscriber = ros::Subscriber<std_msgs::Float32>("slider_pos", &subscriber_callback);
}


bool ROSNode::connect() {
    /*  Initialize Ethernet shield
        DHCP version of this function, Ethernet.begin(mac),
        returns int: 1 on successful connection, 0 on failure.
        Other versions don't return anything
    */
    Ethernet.begin(MAC, IP, DNS, GATEWAY, SUBNET);

    // Check for hardware errors
    if (static_cast<int>(Ethernet.hardwareStatus()) == static_cast<int>(EthernetNoHardware)) {
        Serial.println(F("ERROR: Ethernet shield was not found."));
    }

    if (static_cast<int>(Ethernet.hardwareStatus()) == static_cast<int>(LinkOFF)) {
        Serial.println(F("ERROR: Ethernet cable is not connected."));
    }

    Serial.println(F("Network information:"));
    Serial.print(F("IPv4 Address: "));
    Serial.println(Ethernet.localIP());
    Serial.println(F("Gateway: "));
    Serial.println(Ethernet.gatewayIP());
    Serial.print(F("Subnet mask: "));
    Serial.println(Ethernet.subnetMask());
    Serial.print(F("DNS Server: "));
    Serial.println(Ethernet.dnsServerIP());
    
    nh.getHardware()->setConnection(server, local_port);
    

    // `maintain()` must be frequently called to maintain DHCP lease for the given IP address
    Ethernet.maintain();

}


void ROSNode::subscriber_callback(const std_msgs::Float32& msg_f32){
    
};
