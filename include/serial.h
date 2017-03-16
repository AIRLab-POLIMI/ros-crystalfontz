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

#include  <string>

int Serial_Init(std::string devname, int baud_rate);
void Uninit_Serial();
void SendByte(unsigned char datum);
void Sync_Read_Buffer(void);
dword BytesAvail(void);
ubyte GetByte(void);
dword PeekBytesAvail(void);
void Sync_Peek_Pointer(void);
void AcceptPeekedData(void);
ubyte PeekByte(void);
void SendData(unsigned char *data,int length);
void SendString(char *data);
//============================================================================
