//
//  GeneraMoveStub.cc
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

#ifndef OPENR_STUBGEN
#define OPENR_STUBGEN
#endif

#if defined(__GNUC__)
#include <apsys.h>
#endif
#include <MCOOP.h>
#include <ObjectEntryTable.h>
#include <OPENR/stub_macro.h>
#include "def.h"
#include "entry.h"
#include "GeneraMoveStub.h"

//
//  Object Core
//
#include "GeneraMove.h"
GeneraMove Self;

//
//  Stub Function Definitions
//
StubFuncDefine_Basic(Init)
StubFuncDefine_Basic(Start)
StubFuncDefine_Basic(Stop)
StubFuncDefine_Basic(Destroy)
StubFuncDefine_Control_nofunc(0)
StubFuncDefine_Ready(0, Ready)
StubFuncDefine_Connect_nofunc(0)
StubFuncDefine_Notify(0, GetCamera)

//
// Other definitions if any
//
extern "C" void
_TimerEnd(void* msg)
{
    Self.TimerEnd(msg);
    Return();
}


//
// Set Entry Table
//
#if defined(__GNUC__)
GEN_ENTRY(_hookstub0, _Init);
GEN_ENTRY(_hookstub1, _Start);
GEN_ENTRY(_hookstub2, _Stop);
GEN_ENTRY(_hookstub3, _Destroy);
GEN_ENTRY(_controlstub0, 	_Control0);
GEN_ENTRY(_readystub0, 	_Ready0);
GEN_ENTRY(_connectstub0, 	_Connect0);
GEN_ENTRY(_notifystub0, 	_Notify0);
GEN_ENTRY(_TimerEndstub, 	_TimerEnd);
GEN_ENTRY(PrologueEntry, Prologue);

ObjectEntry	ObjectEntryTable[] = {
    {Entry_Hook[0], 	(Entry)_hookstub0},
    {Entry_Hook[1], 	(Entry)_hookstub1},
    {Entry_Hook[2], 	(Entry)_hookstub2},
    {Entry_Hook[3], 	(Entry)_hookstub3},
    {Entry_Control[0], 	(Entry)_controlstub0},
    {Entry_Ready[0], 	(Entry)_readystub0},
    {Entry_Connect[0], 	(Entry)_connectstub0},
    {Entry_Notify[0], 	(Entry)_notifystub0},
    {Extra_Entry[0], 	(Entry)_TimerEndstub},
    {UNDEF,            (Entry) ENTRY_UNDEF}
};
#else

//
//  Stub Function Pointer
//  
_Hook _hook[numOfHook] = { _Hook(Init), _Hook(Start), _Hook(Stop), _Hook(Destroy) };
_Control _control[numOfSubject]  = { _Control(0) };
_Ready   _ready  [numOfSubject]  = { _Ready(0) };
_Connect _connect[numOfObserver] = { _Connect(0) };
_Notify  _notify [numOfObserver] = { _Notify(0) };

ObjectEntry  ObjectEntryTable[] = {
    {Entry_Hook[0], 	(Entry) _Hook(Init)},
    {Entry_Hook[1], 	(Entry) _Hook(Start)},
    {Entry_Hook[2], 	(Entry) _Hook(Stop)},
    {Entry_Hook[3], 	(Entry) _Hook(Destroy)},
    {Entry_Control[0], 	(Entry) _Control(0)},
    {Entry_Ready[0], 	(Entry) _Ready(0)},
    {Entry_Connect[0], 	(Entry) _Connect(0)},
    {Entry_Notify[0], 	(Entry) _Notify(0)},
    {Extra_Entry[0], 	(Entry) _TimerEnd},
    {UNDEF, 		(Entry) ENTRY_UNDEF}
};
#endif
