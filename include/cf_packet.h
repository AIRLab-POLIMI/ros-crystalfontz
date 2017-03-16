//============================================================================
// This file is part of ros-crystalfontz
//
// Copyright (C) 2017 Artificial Intelligence & Robotics Laboratory
// of Politecnico di Milano (AIRLab) <admin.airlab-deib@polimi.it>
// Distributed under the GNU General Public License version 3.
//
// Special permission to use ros-crystalfontz under the conditions of a 
// different license can be requested from the author.
//============================================================================

#define MAX_DATA_LENGTH 22
//============================================================================
#define MAX_COMMAND 35
//============================================================================
typedef struct
  {
  ubyte
    command;
  ubyte
    data_length;
  ubyte
    data[MAX_DATA_LENGTH];
  WORD_UNION
    CRC;
  }COMMAND_PACKET;
//============================================================================
word get_crc(ubyte *bufptr,word len,word seed);
//============================================================================
extern COMMAND_PACKET
  incoming_command;
extern COMMAND_PACKET
  outgoing_response;
//============================================================================
ubyte check_for_packet(void);
void send_packet(void);
//============================================================================
