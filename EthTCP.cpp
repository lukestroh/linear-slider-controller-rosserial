/*
    Source file for custom Ethernet class
    Author: Luke Strohbehn
    Date: 4/17/2023
*/

#include "EthTCP.h"

Eth::Eth() {
    __init__(); 
}

void Eth::__init__() {
    /* Set class variables */
    server = IPAddress(169, 254, 235, 35);
    local_port = 10002;
    client = EthernetClient();  
}

bool Eth::connect() {
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
    
    if (!client.connected()) {
        if (client.connect(server, local_port) == 1) {
            Serial.println(F("Connected to host.\n"));
        }
        else {
            Serial.println(F("Connection failed.\n"));
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
    }

    // `maintain()` must be frequently called to maintain DHCP lease for the given IP address
    Ethernet.maintain();

    if (client.connected()) {
        return true;
    } else {
        return false;
    }
}

void Eth::read_data() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (client.available() > 0 && newData == false) {
        rc = client.read();
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1; // don't pass max index value
                }
            } else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        } else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void Eth::send_data(int data) { 
    client.println(data);
}
