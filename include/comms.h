//
// Created by gianluca on 5/18/23.
//

#ifndef BUILD_COMMS_H
#define BUILD_COMMS_H

#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <cstdlib>
#include <string>

#include "typedefs.h"
#include "serial.h"
#include "cf_packet.h"
#include "show_packet.h"

// Display state variables
//		All variables are updated by the topics and periodically the dysplay is updated with these values.

// CFA-635 communications protocol only allows
// one outstanding packet at a time. Wait for the response
// packet from the CFA-635 before sending another packet.
void sendPacket(int command){
    outgoing_response.command = command;
    outgoing_response.data_length = 0;
    send_packet();
}
void sendPacket(int command, int d1){
    outgoing_response.command = command;
    outgoing_response.data[0] = d1;
    outgoing_response.data_length = 1;
    send_packet();
}
void sendPacket(int command, int d1, int d2){
    outgoing_response.command = command;
    outgoing_response.data[0] = d1;
    outgoing_response.data[1] = d2;
    outgoing_response.data_length = 2;
    send_packet();
}
void sendPacket(int command, int d1, int d2, char* line){
    outgoing_response.command = command;
    outgoing_response.data[0] = d1;
    outgoing_response.data[1] = d2;
    memcpy(&outgoing_response.data[2], line, 20); // Hardcoded packet length
    outgoing_response.data_length = 22;
    send_packet();
}

// Waits a maximum of 300ms for a packet (an ack, after sending a command)
bool waitForAck(){
    bool timed_out = true;
    for (int k = 0; k <= 300; k++){
        if (check_for_packet()){
            timed_out = false;
            break;
        }
        usleep(1000);
    }
    return !timed_out;
}

#endif //BUILD_COMMS_H
