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

typedef unsigned char ubyte;
typedef signed char sbyte;
typedef unsigned short word;
typedef unsigned long dword;
typedef union
  {
  ubyte
    as_bytes[2];
  word
    as_word;
  } WORD_UNION;
//============================================================================
