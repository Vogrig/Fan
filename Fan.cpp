////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Fan.cpp
//
// Copyright (C) <2016>
//
// by Andrea Vogrig
// http://www.typedef.altervista.org
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

#include "SC_PlugIn.h"
#include <math.h>
#include <string.h>
#include <IOKit/IOKitLib.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>     /* strtof */
#include <iostream>
#include <libkern/OSAtomic.h>

using std::cout;
using std::endl;

static InterfaceTable *ft;

// Period in seconds between sampling data -- this is a CPU-intensive
// operation, and so should not be done once per block. In any case, the
// FAN only appears to update a few times per second.
#define FAN_PERIOD 0.05


#define VERSION               "0.01"

#define KERNEL_INDEX_SMC      2

#define SMC_CMD_READ_BYTES    5
#define SMC_CMD_WRITE_BYTES   6
#define SMC_CMD_READ_INDEX    8
#define SMC_CMD_READ_KEYINFO  9
#define SMC_CMD_READ_PLIMIT   11
#define SMC_CMD_READ_VERS     12

#define DATATYPE_FPE2         "fpe2"
#define DATATYPE_UINT8        "ui8 "
#define DATATYPE_UINT16       "ui16"
#define DATATYPE_UINT32       "ui32"
#define DATATYPE_SP78         "sp78"

// key values
#define SMC_KEY_CPU_TEMP      "TC0P"

static char FAN[5] = "FNum";

static io_connect_t conn;

typedef	kern_return_t		IOReturn;


typedef struct {
    char                  major;
    char                  minor;
    char                  build;
    char                  reserved[1]; 
    UInt16                release;
} SMCKeyData_vers_t;

typedef struct {
    UInt16                version;
    UInt16                length;
    UInt32                cpuPLimit;
    UInt32                gpuPLimit;
    UInt32                memPLimit;
} SMCKeyData_pLimitData_t;

typedef struct {
    UInt32                dataSize;
    UInt32                dataType;
    char                  dataAttributes;
} SMCKeyData_keyInfo_t;

typedef char              SMCBytes_t[32]; 

typedef struct {
  UInt32                  key; 
  SMCKeyData_vers_t       vers; 
  SMCKeyData_pLimitData_t pLimitData;
  SMCKeyData_keyInfo_t    keyInfo;
  char                    result;
  char                    status;
  char                    data8;
  UInt32                  data32;
  SMCBytes_t              bytes;
} SMCKeyData_t;

typedef char              UInt32Char_t[5];

typedef struct {
  UInt32Char_t            key;
  UInt32                  dataSize;
  UInt32Char_t            dataType;
  SMCBytes_t              bytes;
} SMCVal_t;

// BASIC ADMINISTRATION

// InterfaceTable contains pointers to functions in the host (server).
// variable should be called ft because of later use in a macro


// declare struct to hold unit generator state
// Fan is a subclass (derived class) from Unit

struct Fan : public Unit
{
    int count;
	float speed[2]; 
    float minspeed[2];
    float maxspeed[2];
    int sample_counter;
};

// declare unit generator functions
extern "C"
{
	void Fan_next(Fan *unit, int inNumSamples);
	void Fan_Ctor(Fan* unit);
	void Fan_Dtor(Fan* unit);

	// prototypes
	//kern_return_t SMCSetFanRpm(char *key, int rpm);
	kern_return_t SMCPrintFans(Fan* unit);
    kern_return_t SMCUpdateFans(Fan* unit);
	kern_return_t SMCOpen(void);
	kern_return_t SMCClose();
	kern_return_t SMCCall(int index, SMCKeyData_t *inputStructure, SMCKeyData_t *outputStructure);
	kern_return_t SMCReadKey(UInt32Char_t key, SMCVal_t *val);
	int SMCGetFanRpm(char *key);
	UInt32 _strtoul(char *str, int size, int base);
	float _strtof(unsigned char *str, int size, int e);
	void _ultostr(char *str, UInt32 val);
};

//////////////////////////////////////////////////////////////////

// CONSTRUCTOR
// every UGen needs a constructor (usually referred to as a Ctor)

// Ctor is called to initialize the unit generator.
// It only executes once.

// A Ctor usually does 3 things.
// 1. set the calculation function.
// 2. initialize the unit generator state variables.
// 3. calculate one sample of output.

// Ctor should be called UGenName_Ctor because macro DefineSimpleUnit assumes that name

void Fan_Ctor(Fan* unit)
{

	// 1. set the calculation function.
	SETCALC(Fan_next);

	// 2. initialize the unit generator state variables.

    unit->count = 0;
    unit->speed[0] = 0;
    unit->minspeed[0] = 0;
    unit->maxspeed[0] = 0;
    unit->speed[1] = 0;
    unit->minspeed[1] = 0;
    unit->maxspeed[1] = 0;
    unit->sample_counter = 0;

	SMCOpen();

    SMCPrintFans(unit);

    if(IN0(0) == 1)//normalized
        unit->speed[0] = ((unit->speed[0] - unit->minspeed[0])/unit->maxspeed[0]); 
	
	// 3. calculate one sample of output.
    
    Fan_next(unit, 1);

}


void Fan_Dtor(Fan* unit)
{
	SMCClose();
}
//////////////////////////////////////////////////////////////////

// UGEN CALCULATION

void Fan_next(Fan *unit, int inNumSamples)
{
    
    if (--unit->sample_counter < 0){  

        SMCUpdateFans(unit);
       
        unit->sample_counter = FAN_PERIOD *
        unit->mWorld->mSampleRate /
        unit->mWorld->mFullRate.mBufLength;
    }
    
    if(IN0(0) == 1){//normalized
        unit->speed[0] = ((unit->speed[0] - unit->minspeed[0])/unit->maxspeed[0]); 
        if(unit->count > 1)
            unit->speed[1] = ((unit->speed[1] - unit->minspeed[1])/unit->maxspeed[1]); 
    }
    
    ZOUT0(0) = unit->speed[0];
    if(unit->count > 1)
        ZOUT0(1) = unit->speed[1];
    
}

////////////////////////////////////////////////////////////////////


// LOAD FUNCTION
PluginLoad(Fan)
{
    ft = inTable;  // ft was declared above

    DefineSimpleUnit(Fan);
}

////////////////////////////////////////////////////////////////////

UInt32 _strtoul(char *str, int size, int base)
{
    UInt32 total = 0;
    int i;

    for (i = 0; i < size; i++)
    {
        if (base == 16)
            total += str[i] << (size - 1 - i) * 8;
        else
           total += (unsigned char) (str[i] << (size - 1 - i) * 8);
    }
    return total;
}

void _ultostr(char *str, UInt32 val)
{
    str[0] = '\0';
    sprintf(str, "%c%c%c%c", 
            (unsigned int) val >> 24,
            (unsigned int) val >> 16,
            (unsigned int) val >> 8,
            (unsigned int) val);
}


float _strtof(unsigned char *str, int size, int e)
{
    float total = 0;
    int i;
    
    for (i = 0; i < size; i++)
    {
        if (i == (size - 1))
            total += (str[i] & 0xff) >> e;
        else
            total += str[i] << (size - 1 - i) * (8 - e);
    }
    
	total += (str[size-1] & 0x03) * 0.25;
    
    return total;
}

kern_return_t SMCOpen(void)
{
    kern_return_t result;
    io_iterator_t iterator;
    io_object_t   device;

    CFMutableDictionaryRef matchingDictionary = IOServiceMatching("AppleSMC");
    result = IOServiceGetMatchingServices(kIOMasterPortDefault, matchingDictionary, &iterator);
    if (result != kIOReturnSuccess)
    {
        printf("Error: IOServiceGetMatchingServices() = %08x\n", result);
        return 1;
    }

    device = IOIteratorNext(iterator);
    IOObjectRelease(iterator);
    if (device == 0)
    {
        printf("Error: no SMC found\n");
        return 1;
    }

    result = IOServiceOpen(device, mach_task_self(), 0, &conn);
    IOObjectRelease(device);
    if (result != kIOReturnSuccess)
    {
        printf("Error: IOServiceOpen() = %08x\n", result);
        return 1;
    }

    return kIOReturnSuccess;
}

kern_return_t SMCClose()
{
    return IOServiceClose(conn);
}


kern_return_t SMCCall(int index, SMCKeyData_t *inputStructure, SMCKeyData_t *outputStructure)
{
    size_t   structureInputSize;
    size_t   structureOutputSize;

    structureInputSize = sizeof(SMCKeyData_t);
    structureOutputSize = sizeof(SMCKeyData_t);

    #if MAC_OS_X_VERSION_10_5
    return IOConnectCallStructMethod( conn, index,
                            // inputStructure
                            inputStructure, structureInputSize,
                            // ouputStructure
                            outputStructure, &structureOutputSize );
    #else
    return IOConnectMethodStructureIStructureO( conn, index,
                                                structureInputSize, /* structureInputSize */
                                                &structureOutputSize,   /* structureOutputSize */
                                                inputStructure,        /* inputStructure */
                                                outputStructure);       /* ouputStructure */
    #endif

}

kern_return_t SMCReadKey(UInt32Char_t key, SMCVal_t *val)
{
    kern_return_t result;
    SMCKeyData_t  inputStructure;
    SMCKeyData_t  outputStructure;

    memset(&inputStructure, 0, sizeof(SMCKeyData_t));
    memset(&outputStructure, 0, sizeof(SMCKeyData_t));
    memset(val, 0, sizeof(SMCVal_t));

    inputStructure.key = _strtoul(key, 4, 16);
    inputStructure.data8 = SMC_CMD_READ_KEYINFO;

    result = SMCCall(KERNEL_INDEX_SMC, &inputStructure, &outputStructure);
    if (result != kIOReturnSuccess)
        return result;

    val->dataSize = outputStructure.keyInfo.dataSize;
    _ultostr(val->dataType, outputStructure.keyInfo.dataType);
    inputStructure.keyInfo.dataSize = val->dataSize;
    inputStructure.data8 = SMC_CMD_READ_BYTES;

    result = SMCCall(KERNEL_INDEX_SMC, &inputStructure, &outputStructure);
    if (result != kIOReturnSuccess)
        return result;

    memcpy(val->bytes, outputStructure.bytes, sizeof(outputStructure.bytes));

    return kIOReturnSuccess;
}

kern_return_t SMCPrintFans(Fan* unit)
{
    kern_return_t result;
    SMCVal_t      val;
    UInt32Char_t  key;
    int             i=0;
    
    result = SMCReadKey(FAN, &val);
    if (result != kIOReturnSuccess)
        return kIOReturnError;
    
    unit->count = _strtoul((char *)val.bytes, val.dataSize, 10);

    // unit->speeds =   (float*)RTAlloc(unit->mWorld, unit->count * sizeof(float));
    // unit->minspeeds = (float*)RTAlloc(unit->mWorld, unit->count * sizeof(float));
    // unit->maxspeeds = (float*)RTAlloc(unit->mWorld, unit->count * sizeof(float));

    for (i = 0; i < unit->count; ++i){
        //printf("\nFan #%d:\n", i);
        // sprintf(key, "F%dID", i);
        // SMCReadKey(key, &val);
        //printf("    Fan ID       : %s\n", val.bytes+4);
        sprintf(key, "F%dAc", i); 
        SMCReadKey(key, &val); 
        //printf("    Actual speed : %.0f\n", _strtof((unsigned char *)val.bytes, val.dataSize, 2));

        // *unit->speeds++ = _strtof((unsigned char *)val.bytes, val.dataSize, 2);
        unit->speed[i] = _strtof((unsigned char *)val.bytes, val.dataSize, 2);

        sprintf(key, "F%dMn", i);
        SMCReadKey(key, &val);
        //printf("    Minimum speed: %.0f\n", _strtof((unsigned char *)val.bytes, val.dataSize, 2));
        
        // *unit->minspeeds++ = _strtof((unsigned char *)val.bytes, val.dataSize, 2);
        unit->minspeed[i] = _strtof((unsigned char *)val.bytes, val.dataSize, 2);

        sprintf(key, "F%dMx", i);
        SMCReadKey(key, &val);
        
        //printf("    Maximum speed: %.0f\n", _strtof((unsigned char *)val.bytes, val.dataSize, 2));
        // *unit->maxspeeds++ = _strtof((unsigned char *)val.bytes, val.dataSize, 2);
        unit->maxspeed[i] = _strtof((unsigned char *)val.bytes, val.dataSize, 2);
        // sprintf(key, "F%dSf", i);
        // SMCReadKey(key, &val);
        // printf("    Safe speed   : %.0f\n", _strtof((unsigned char *)val.bytes, val.dataSize, 2));
        // sprintf(key, "F%dTg", i);
        // SMCReadKey(key, &val);
        // printf("    Target speed : %.0f\n", _strtof((unsigned char *)val.bytes, val.dataSize, 2));
        // SMCReadKey("FS! ", &val);
        //  if ((_strtoul((char *)val.bytes, 2, 16) & (1 << i)) == 0)
        //      printf("    Mode         : auto\n");
        //  else
        //      printf("    Mode         : forced\n");
     }
    
    return kIOReturnSuccess;
}

kern_return_t SMCUpdateFans(Fan* unit)
{
    kern_return_t result;
    SMCVal_t      val;
    UInt32Char_t  key;
    int           i;
    
    for (i = 0; i < unit->count; ++i){
       
        sprintf(key, "F%dAc", i); 
        result = SMCReadKey(key, &val);
        if (result != kIOReturnSuccess)
            return kIOReturnError;
        unit->speed[i] = _strtof((unsigned char *)val.bytes, val.dataSize, 2);

    }
    
    return kIOReturnSuccess;
}


////////////////////////////////////////////////////////////////////
