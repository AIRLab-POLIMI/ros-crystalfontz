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

#include  <termios.h>
#include  <unistd.h>
#include  <fcntl.h>
#include <iostream>
#include  "typedefs.h"
#include  "serial.h"

// com port handle
int handle;

// data buffering
#define RECEIVEBUFFERSIZE 4096
ubyte SerialReceiveBuffer[RECEIVEBUFFERSIZE];
dword ReceiveBufferHead;
dword ReceiveBufferTail;
dword ReceiveBufferTailPeek;

//------------------------------------------------------------------------------
int Serial_Init(const std::string& devname, int baud_rate)
  {
  int
    brate;
  struct
    termios term;

  //open device
  handle = open(devname.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if(handle <= 0)
    {
//		ROS_INFO("Serial_Init:: Open() failed\n");
		return(1);
    }

  //get baud rate constant from numeric value
  switch (baud_rate)
  {
    case 19200:
      brate=B19200;
      break;
    case 115200:
      brate=B115200;
      break;
    default:
      std::cerr<<"Serial_Init:: Invalid baud rate: "<<baud_rate<<" (must be 19200 or 115200)"<<std::endl;
      return(2);
  }
  //get device struct
  if(tcgetattr(handle, &term) != 0)
    {
    std::cerr<<"Serial_Init:: tcgetattr() failed"<<std::endl;
    return(3);
    }

  //input modes
  term.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|INPCK|ISTRIP|INLCR|IGNCR|ICRNL
                  |IXON|IXOFF);
  term.c_iflag |= IGNPAR;

  //output modes
  term.c_oflag &= ~(OPOST|ONLCR|OCRNL|ONOCR|ONLRET|OFILL
                  |OFDEL|NLDLY|CRDLY|TABDLY|BSDLY|VTDLY|FFDLY);

  //control modes
  term.c_cflag &= ~(CSIZE|PARENB|PARODD|HUPCL|CRTSCTS);
  term.c_cflag |= CREAD|CS8|CSTOPB|CLOCAL;

  //local modes
  term.c_lflag &= ~(ISIG|ICANON|IEXTEN|ECHO);
  term.c_lflag |= NOFLSH;

  //set baud rate
  cfsetospeed(&term, brate);
  cfsetispeed(&term, brate);

  //set new device settings
  if(tcsetattr(handle, TCSANOW, &term)  != 0)
  {
      std::cerr<<"Serial_Init:: tcsetattr() failed"<<std::endl;
    return(4);
    }

  ReceiveBufferHead=ReceiveBufferTail=0;

//  ROS_INFO("Serial_Init:: success");
  return(0);
  }
//------------------------------------------------------------------------------
void Uninit_Serial()
  {
  close(handle);
  handle=0;
  }
//------------------------------------------------------------------------------
void SendByte(unsigned char datum)
  {
  dword
    bytes_written;
  bytes_written=0;

  if(handle)
    {
    if((bytes_written=write(handle, &datum, 1)) != 1)
        std::cerr<<"SendByte(): system call write() return error."<<std::endl<<"Asked for 1 bytes to be written, but "<<bytes_written<<" bytes reported as written."<<std::endl;
    }
  else
      std::cerr<<"SendByte(): handle is null"<<std::endl;
  }
//------------------------------------------------------------------------------
void SendData(unsigned char *data,int length)
  {
  dword
  bytes_written=0;

  if(handle)
    {
      if((bytes_written=write(handle, data, length)) != length)
      {
//      ROS_INFO("SendData(): system call \"write()\" return error.\n  Asked for %d bytes to be written, but %d bytes reported as written.", length,(int)bytes_written);    
      }
    }
  else
      std::cerr<<"SendData(): handle is null"<<std::endl;

  }
//------------------------------------------------------------------------------
void SendString(char *data)
  {
  while(*data)
    {
    usleep(500);
    SendByte(*data++);
    }
  }
//------------------------------------------------------------------------------
//Gets incoming data and puts it into SerialReceiveBuffer[].
void Sync_Read_Buffer()
  {
  ubyte
    Incoming[4096];
  long
    BytesRead;

  //  COMSTAT status;
  dword
    i;

  if(!handle)
    return;

  //get data
  BytesRead = read(handle, &Incoming, 4096);
  if(0<BytesRead)
  {
  //Read the incoming ubyte, store it.
  for(i=0; i < BytesRead; i++)
    {
    SerialReceiveBuffer[ReceiveBufferHead] = Incoming[i];

    //Increment the pointer and wrap if needed.
    ReceiveBufferHead++;
    if (RECEIVEBUFFERSIZE <= ReceiveBufferHead)
      ReceiveBufferHead=0;
      }
    }
  }
/*---------------------------------------------------------------------------*/
dword BytesAvail()
  {
  //LocalReceiveBufferHead and return_value are a signed variables.
  int
    LocalReceiveBufferHead;
  int
    return_value;

  //Get a register copy of the head pointer, since an interrupt that
  //modifies the head pointer could screw up the value. Convert it to
  //our signed local variable.
  LocalReceiveBufferHead = ReceiveBufferHead;
  if((return_value = (LocalReceiveBufferHead - (int)ReceiveBufferTail)) < 0)
    return_value+=RECEIVEBUFFERSIZE;

  return(return_value);
  }
/*---------------------------------------------------------------------------*/
ubyte GetByte()
  {
  dword
    LocalReceiveBufferTail;
  dword
    LocalReceiveBufferHead;
  ubyte
    return_byte=0;

  //Get a register copy of the tail pointer for speed and size.
  LocalReceiveBufferTail=ReceiveBufferTail;

  //Get a register copy of the head pointer, since an interrupt that
  //modifies the head pointer could screw up the value.
  LocalReceiveBufferHead=ReceiveBufferHead;


  //See if there are any more bytes available.
  if(LocalReceiveBufferTail!=LocalReceiveBufferHead)
    {
    //There is at least one more ubyte.
    return_byte=SerialReceiveBuffer[LocalReceiveBufferTail];

    //Increment the pointer and wrap if needed.
    LocalReceiveBufferTail++;
    if(RECEIVEBUFFERSIZE<=LocalReceiveBufferTail)
      LocalReceiveBufferTail=0;

    //Update the real ReceiveBufferHead with our register copy. Make sure
    //the ISR does not get serviced during the transfer.
    ReceiveBufferTail=LocalReceiveBufferTail;
    }

  return(return_byte);
  }
/*---------------------------------------------------------------------------*/
dword PeekBytesAvail()
  {
  //LocalReceiveBufferHead and return_value are a signed variables.
  int
    LocalReceiveBufferHead;
  int
    return_value;

  //Get a register copy of the head pointer, since an interrupt that
  //modifies the head pointer could screw up the value. Convert it to
  //our signed local variable.
  LocalReceiveBufferHead=ReceiveBufferHead;
  if((return_value=(LocalReceiveBufferHead-(int)ReceiveBufferTailPeek))<0)
    return_value+=RECEIVEBUFFERSIZE;
  return(return_value);
  }
/*---------------------------------------------------------------------------*/
void Sync_Peek_Pointer()
  {
  ReceiveBufferTailPeek=ReceiveBufferTail;
  }
/*---------------------------------------------------------------------------*/
void AcceptPeekedData()
  {
  ReceiveBufferTail=ReceiveBufferTailPeek;
  }
/*---------------------------------------------------------------------------*/
ubyte PeekByte()
  {
  int
    LocalReceiveBufferTailPeek;
  int
    LocalReceiveBufferHead;
  ubyte return_byte=0;

  //Get a register copy of the tail pointer for speed and size.
  LocalReceiveBufferTailPeek=ReceiveBufferTailPeek;

  //Get a register copy of the head pointer, since an interrupt that
  //modifies the head pointer could screw up the value.
  LocalReceiveBufferHead=ReceiveBufferHead;

  //See if there are any more bytes available.
  if(LocalReceiveBufferTailPeek!=LocalReceiveBufferHead)
    {
    //There is at least one more ubyte.
    return_byte=SerialReceiveBuffer[LocalReceiveBufferTailPeek];

    //Increment the pointer and wrap if needed.
    LocalReceiveBufferTailPeek++;
    if(RECEIVEBUFFERSIZE<=LocalReceiveBufferTailPeek)
      LocalReceiveBufferTailPeek=0;

    //Update the real ReceiveBufferHead with our register copy. Make sure
    //the ISR does not get serviced during the transfer.
    ReceiveBufferTailPeek=LocalReceiveBufferTailPeek;
    }
  return(return_byte);
  }
/*---------------------------------------------------------------------------*/
