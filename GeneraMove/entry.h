//
//  entry.h
//
//  This file is generated by stubgen2.
//

//
// Copyright 2002 Sony Corporation 
//
// Permission to use, copy, modify, and redistribute this software for
// non-commercial use is hereby granted.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//

#ifndef _entry_h_DEFINED
#define _entry_h_DEFINED

#include <Types.h>
#include "def.h"

const int numOfHook = 4;

const Selector Entry_Hook   [numOfHook]     = { 0, 1, 2, 3 };
const Selector Entry_Control[numOfSubject]  = { 4 };
const Selector Entry_Ready  [numOfSubject]  = { 5 };
const Selector Entry_Connect[numOfObserver] = { 6 };
const Selector Entry_Notify [numOfObserver] = { 7 };

//
//  Number of basic entries is 
//      numOfHook + (numOfSubject + numOfObserver) * 2
//
const longword numOfBasicEntries = 8;

#endif // _entry_h_DEFINED
