# ECMC knowledgebase:

## Error codes (quick guide):
Item | Error | Error Id | When | Description
--- | --- | --- | --- | --- |
1   | ERROR_EC_MAIN_DOMAIN_DATA_FAILED | 0x2600c | At "Cfg.SetAppMode(1)" | No hardware configured. Missing "addSlave.cmd" or missing "Cfg.EcApplyConfig(1)"
2   | ERROR_MAIN_EC_ACTIVATE_FAILED    | 0x2001b | At "Cfg.SetAppMode(1)" | No hardware configured. Missing "addSlave.cmd" or missing "Cfg.EcApplyConfig(1)"
3   | ERROR_MON_BOTH_LIMIT_INTERLOCK    | 0x14c10 | In runtime | See "Lost frames. Lost Slaves, ECMC in error state" below.
4   | ERROR_AXIS_HARDWARE_STATUS_NOT_OK    | 0x14315 | In runtime | See "Lost frames. Lost Slaves, ECMC in error state" below.
5   | ERROR_EC_SLAVE_NOT_OPERATIONAL    | 0x24011 | In runtime | See "Lost frames. Lost Slaves, ECMC in error state" below.
6   | ERROR_EC_SLAVE_NOT_ONLINE    | 0x24012 | In runtime | See "Lost frames. Lost Slaves, ECMC in error state" below.
7   | ERROR_EC_LINK_DOWN    | 0x26011 | In runtime | Link is down. Could be lost physical connection. Ensure that cable is connected. See "Lost frames. Lost Slaves, ECMC in error state" below.


## Lost frames, lost slaves, ECMC in error state
### Issues: 
* EtherCAT frames are lost resulting in ecmc alarm state.
* Connection to slaves are lost resulting in ecmc alarm state
* Error codes:
  * ERROR_MON_BOTH_LIMIT_INTERLOCK (0x14c10)
  * ERROR_AXIS_HARDWARE_STATUS_NOT_OK (0x14315)
  * ERROR_EC_SLAVE_NOT_OPERATIONAL (0x24011)
  * ERROR_EC_SLAVE_NOT_ONLINE (0x24012)
  * ERROR_EC_LINK_DOWN (0x26011)

Note: These error codes always appear when starting ecmc. This is because it can take up to 30s untill the bus in syncronized and stable. After this startup you should see a printout stating "NO_ERROR". If any of the above error appear after the system have been started up then please see below for suggestions.

### Remedy:
1. Check cabling. There have also been cases where cabling have failed resulting in intermittent failures.
2. Make sure you run rt-patch (for more info see repo: https://github.com/icshwi/realtime-config)
3. Check NIC perfromance (with "iperf"). There have been cases where NIC have been broken which took a significat time to find.
4. Measure ecmc jitter:
  * camonitor ${P}MCU-thread-latency-max | tee jitter.log
  * Unit of jitter in log file is nano seconds
  * Jitter above 100 micro seconds is to be considered to be high and should be avoided.

## Slaves in error state, strange behaviour when using ethercat slaves, ethercat rescan
### Issues: 
* Errors in dmesg, 
  * Problem reading SII, 
  * Timeouts
  * Much more.....
* Slaves stuck in "INIT E" 
* Somtimes succeded when ethercat rescan but not consistent failed again.
* Slaves in error state
* A few times the bus was failing soon again.

### Remedy:
* Seemed issue was fundamental, so needed complete reinstall (Centos, realtime-config etherlabmaster, e3)
* Did not work with only reinstall of etherlabmaster
* After reinstall everything worked properly.

## Fast callbacks on waveforms

If excpetced callbacks are failing it could usefull to read the below text:

From EPICS tech talk 2014: https://epics.anl.gov/tech-talk/2014/msg00284.php

Subject: 	RE: Waveform record I/O interrupt. asyn
From: 	Mark Rivers <rivers@cars.uchicago.edu>
To: 	"'Vishnu Patel'" <vishnu.patel@rediffmail.com>, "techtalk " <tech-talk@aps.anl.gov>
Date: 	Fri, 21 Feb 2014 23:06:57 +0000

This was my previous response to them on this topic, which was not copied to tech-talk.
If I have mis-stated anything I hope someone will correct me.
> In my waveform record 3 elements are there and any one of the value changed it will report to change value.
> So for example the set of value is 5,6,1 and then change to 5,6,0 it will report to value change and I/O Intr activate. 
> 
> The expected camonitor output should be as below
>       $ camonitor PV
>       PV   5 6 1
>       PV   5 6 0
> 
>  But sometime it is
> 
>     $ camonitor PV
>       PV   5 6 0
>       PV   5 6 0
>
This is actually the expected behavior.  Here is the order of operations:
1) Driver does callback to waveform record device support
2) Device support callback function:
  2a) Locks record
  2b) Copies array into record
  2c) Unlocks record
  2d) Calls scanIORequest to have a callback thread to process the record
3) Callback thread processes the record, sending out Channel Access monitors, etc.
 
What you are seeing is:
1) Your driver has done a callback, copying [5,6,1] into the record.  It has requested a callback to process the record with scanIORequest
2) Before the scanIORequest thread has a chance to process the record in step 3 above, your driver does a second callback.  This copies [5,6,0] into the record and calls scanIORequest again.
3) The first scanIORequest processes the record.  The data is now [5,6,0] so it posts a monitor with that.
4) The second scanIORequest processes the record, again posting [5,6,0]
 
This is normal behavior.  EPICS does not guarantee that the record will post new monitors on every change if they happen too close together.

For non-array records the asyn device support has a "ring buffer" that stores a certain number of callback values from the driver.  The default ring buffer size is 10.  In this case if several driver callbacks happen faster than the scanIORequest can complete the record will post monitors on all of them.  But eventually if the callbacks happen too fast the ring buffer will overflow and values will be lost.

For array records like you are using there is no ring buffer, because the size could be very large and it would involve copying a large amount of data.  So for arrays if there is a second callback before the first scanIORequest completes you only see the value from the second callback.

The bug that I fixed in asyn 4.21 was that the record was not locked while the data was being copied.  This meant that the record could post a monitor with data that was partly from callback 1 and partly from callback 2.  This was a bad behavior, but that has been fixed.  Now the record data is guaranteed to either be the complete array from callback 1 or callback 2, and not a mixture.

Mark
