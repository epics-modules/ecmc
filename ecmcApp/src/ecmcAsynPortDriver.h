/*
 * ecmcAsynPortDriver.h
 * 
 * Asyn driver that inherits from the asynPortDriver class to demonstrate its use.
 * It simulates a digital scope looking at a 1kHz 1000-point noisy sine wave.  Controls are
 * provided for time/division, volts/division, volt offset, trigger delay, noise amplitude, update time,
 * and run/stop.
 * Readbacks are provides for the waveform data, min, max and mean values.
 *
 * Author: Mark Rivers
 *
 * Created Feb. 5, 2009
 */
#ifndef ASYNPORTDRIVER_H_
#define ASYNPORTDRIVER_H_

#include "asynPortDriver.h"
#include <epicsEvent.h>

class ecmcAsynPortDriver : public asynPortDriver {
public:
    ecmcAsynPortDriver(const char *portName,int paramTableSize,int autoConnect,int priority);
    virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual);
    virtual asynStatus readOctet(asynUser *pasynUser, char *value, size_t maxChars,size_t *nActual, int *eomReason);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus readInt8Array(asynUser *pasynUser, epicsInt8 *value,size_t nElements, size_t *nIn);
    virtual asynStatus readInt16Array(asynUser *pasynUser, epicsInt16 *value,size_t nElements, size_t *nIn);
    virtual asynStatus readInt32Array(asynUser *pasynUser, epicsInt32 *value,size_t nElements, size_t *nIn);
    virtual asynStatus readFloat32Array(asynUser *pasynUser, epicsFloat32 *value,size_t nElements, size_t *nIn);
    virtual asynStatus readFloat64Array(asynUser *pasynUser, epicsFloat64 *value,size_t nElements, size_t *nIn);
    void setAllowRtThreadCom(bool allowRtCom);
    bool getAllowRtThreadCom();
    asynUser *getTraceAsynUser();
protected:
 
private:
    int readArrayGeneric(asynUser *pasynUser, epicsUInt8 *value,size_t nElements, size_t *nIn,size_t typeSize,const char* functionName);
    bool allowRtThreadCom_;
};

#endif /* ASYNPORTDRIVER_H_ */
