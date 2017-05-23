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

#include "asynPortDriver.h"

/** Class that demonstrates the use of the asynPortDriver base class to greatly simplify the task
  * of writing an asyn port driver.
  * This class does a simple simulation of a digital oscilloscope.  It computes a waveform, computes
  * statistics on the waveform, and does callbacks with the statistics and the waveform data itself. 
  * I have made the methods of this class public in order to generate doxygen documentation for them,
  * but they should really all be private. */
class ecmcAsynPortDriver : public asynPortDriver {
public:
    ecmcAsynPortDriver(const char *portName,int paramTableSize,int autoConnect);

    /* These are the methods that we override from asynPortDriver */
    //virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    //virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    //virtual asynStatus readFloat64Array(asynUser *pasynUser, epicsFloat64 *value,
                                        //size_t nElements, size_t *nIn);
    //virtual asynStatus readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[],
                                //size_t nElements, size_t *nIn);

    /* These are the methods that are new to this class */
    void simTask(void);

protected:
 
private:
    /* Our data */
    epicsEventId eventId_;
};
