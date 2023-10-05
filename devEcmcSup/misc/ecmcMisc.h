/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcMisc.h
*
*  Created on: Jan 10, 2019
*      Author: anderssandstrom
*
\*************************************************************************/

/**
\file
    @brief Misc. commands
*/

#ifndef ECMC_MISC_H_
#define ECMC_MISC_H_

#define CHECK_COMMAND_LIST_RETURN_IF_ERROR(commandListIndex)                  \
{                                                                             \
  if (indexCommandList >= ECMC_MAX_COMMANDS_LISTS || indexCommandList < 0) {  \
    LOGERR("ERROR: Command list index out of range.\n");                      \
    return ERROR_COMMAND_LIST_INDEX_OUT_OF_RANGE;                             \
  }                                                                           \
  if (commandLists[indexCommandList] == NULL) {                               \
    LOGERR("ERROR: Command list object NULL.\n");                             \
    return ERROR_COMMAND_LIST_NULL;                                           \
  }                                                                           \
}                                                                             \

#define CHECK_EVENT_RETURN_IF_ERROR(indexEvent)                               \
{                                                                             \
  if (indexEvent >= ECMC_MAX_EVENT_OBJECTS || indexEvent < 0) {               \
    LOGERR("ERROR: Event index out of range.\n");                             \
    return ERROR_MAIN_EVENT_INDEX_OUT_OF_RANGE;                               \
  }                                                                           \
  if (events[indexEvent] == NULL) {                                           \
    LOGERR("ERROR: Event object NULL.\n");                                    \
    return ERROR_MAIN_EVENT_NULL;                                             \
  }                                                                           \
}                                                                             \

#define CHECK_STORAGE_RETURN_IF_ERROR(indexStorage)                           \
{                                                                             \
  if (indexStorage >= ECMC_MAX_DATA_STORAGE_OBJECTS || indexStorage < 0) {    \
    LOGERR("ERROR: Data storage index out of range.\n");                      \
    return ERROR_MAIN_DATA_STORAGE_INDEX_OUT_OF_RANGE;                        \
  }                                                                           \
  if (dataStorages[indexStorage] == NULL) {                                   \
    LOGERR("ERROR: Data storage object NULL.\n");                             \
    return ERROR_MAIN_DATA_STORAGE_NULL;                                      \
  }                                                                           \
}                                                                             \

#define CHECK_RECORDER_RETURN_IF_ERROR(indexRecorder)                         \
{                                                                             \
  if (indexRecorder >= ECMC_MAX_DATA_RECORDERS_OBJECTS || indexRecorder < 0) {\
    LOGERR("ERROR: Data recorder index out of range.\n");                     \
    return ERROR_MAIN_DATA_RECORDER_INDEX_OUT_OF_RANGE;                       \
  }                                                                           \
  if (dataRecorders[indexRecorder] == NULL) {                                 \
    LOGERR("ERROR: Data recorder object NULL.\n");                            \
    return ERROR_MAIN_DATA_RECORDER_NULL;                                     \
  }                                                                           \
}                                                                             \

# ifdef __cplusplus
extern "C" {
# endif  // ifdef __cplusplus


/** \brief Create an event object.
 *
 * An event can be trigger actions based on hardware related events (changes
 * of values on the EtherCAT bus (entries)).\n
 *
 * The events needs to be configured with atleast a subset of the below commands:\n
 * 1. linkEcEntryToEvent()
 * 2. setEventType()
 * 3. setEventSampleTime()
 * 4. setEventTriggerEdge()
 * 5. setEventEnableArmSequence()
 *
 * Currently two kinds of objects can be linked to an event:\n
 * 1. Data recorder object (records data at a triggered event), see
 * command createRecorder().\n
 * 2. Command list object (executes a series of commands at a triggered
 * event), see command  createCommandList().\n
 *
 * \param[in] index Index of event to create.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Create a event object at index 1.\n
 *  "Cfg.CreateEvent(1)" //Command string to ecmcCmdParser.c\n
 */
int createEvent(int index);

/** \brief Links an EtherCAT entry to an event object. \n
 *
 *  \param[in] eventIndex Index of event object to link to.\n
 *  \param[in] eventEntryIndex Index of event objects entry list.\n
 *    eventEntryIndex = 0: Event trigger data input.\n
 *    eventEntryIndex = 1: Arm output (to re-arm latching hardware).\n
 *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
 *    slaveBusPosition = -1: Used to address the simulation slave. Only two
 *                           entries are configured, "ZERO" with default
 *                           value 0 and "ONE" with default value 1.\n
 *    slaveBusPosition = 0..65535: Addressing of normal EtherCAT slaves.\n
 *  \param[in] entryIdString String for addressing purpose (see command
 *                      "Cfg.EcAddEntryComplete() for more information").\n
 *  \param[in] entryBitIndex Bit index of EtherCAT entry to use.\n
 *    entryBitIndex = -1: All bits of the entry will be used.\n
 *    entryBitIndex = 0..64: Only the selected bit will be used.\n
 *
 *
 * \return 0 if success or otherwise an error code.\n
 *
 *  \note Example 1: Link bit 0 of an EtherCAT entry configured as "INPUT_0"
 *  in slave 1 as "event trigger data" for event object 7.\n
 *  "Cfg.LinkEcEntryToEvent(7,0,1,"INPUT_0",0)" //Command string to ecmcCmdParser.c\n
 *
 *   \todo This function have not consistent parameter order with the other
 *    link functions as "linkEcEntryToAxisMon".\n
 */
int linkEcEntryToEvent(int   indexEvent,
                       int   eventEntryIndex,
                       int   slaveBusPosition,
                       char *entryIDString,
                       int   bitIndex);

/** \brief Set event type.\n
 *
 * \param[in] indexEvent Index of event to address.\n
 * \param[in] type Type of event.\n
 *   type = 0: Sample rate triggered (see setEventSampleTime()).\n
 *   type = 1: Trigger on data source edge (see setEventTriggerEdge()).\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set event type to edge triggered for event 5.\n
 *  "Cfg.SetEventType(5,1)" //Command string to ecmcCmdParser.c\n
 */
int setEventType(int indexEvent,
                 int type);

/** \brief Set event sampling time (cycle counts).\n
 *
 * Sets the sample rate at which events will be triggered.
 *
 * Note: This parameter is only valid for sample rate triggered event type (type = 0),
 * see setEventType().
 *
 * \param[in] indexEvent Index of event to address.\n
 * \param[in] type Type of event.\n
 *   type = 0: Sample rate triggered (see setEventSampleTime()).\n
 *   type = 1: Trigger on data source edge (see setEventTriggerEdge()).\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set event sample time (cycle counts) to 10 for event 5.\n
 *  "Cfg.SetEventSampleTime(5,10)" //Command string to ecmcCmdParser.c\n
 */
int setEventSampleTime(int indexEvent,
                       int sampleTime);

/** \brief Set event trigger edge.\n
 *
 * \param[in] indexEvent Index of event to address.\n
 * \param[in] triggerEdge Type of event.\n
 *   triggerEdge = 0: Trigger event on positive edge.\n
 *   triggerEdge = 1: Trigger event on negative edge.\n
 *   triggerEdge = 2: Trigger event on on change.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set event trigger edge to positive.\n
 *  "Cfg.SetEventTriggerEdge(1,0)" //Command string to ecmcCmdParser.c\n
 */
int setEventTriggerEdge(int indexEvent,
                        int triggerEdge);

/** \brief Enable event.\n
 *
 * Event evaluation and triggering is only active when the enable bit is
 * high.\n
 *
 * \param[in] indexEvent Index of event to address.\n
 * \param[in] enable Enable.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Start evaluation of events for event object 4 .\n
 *  "Cfg.SetEventEnable(4,1)" //Command string to ecmcCmdParser.c\n
 */
int setEventEnable(int indexEvent,
                   int enable);

/** \brief Get event enabled.\n
 *
 * Event evaluation and triggering is only active when the enable bit is
 * high.\n
 *
 * \param[in] indexEvent Index of event to address.\n
 * \param[out] enabled Enabled.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get Enabled for event 2 .\n
 *  "GetEventEnabled(2)" //Command string to ecmcCmdParser.c\n
 */
int getEventEnabled(int  indexEvent,
                    int *enabled);

/** \brief Enable arm sequence.\n
 *
 * Some hardware require that the input card is re-armed after each value have
 * been collected (typically latched I/O). The arm sequence will, if enabled,
 * write a 0 followed by a 1 the following cycle to the arm output entry, see
 * command linkEcEntryToEvent().\n
 *
 * \param[in] indexEvent Index of event to address.\n
 * \param[in] enable Enable arm sequence.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable arm sequence for event 3 .\n
 *  "Cfg.SetEventEnableArmSequence(3,1)" //Command string to ecmcCmdParser.c\n
 */
int setEventEnableArmSequence(int indexEvent,
                              int enable);

/** \brief Enable diagnostic printouts from event object.\n
 *
 * \param[in] indexEvent Index of event to address.\n
 * \param[in] enable Enable diagnostic printouts.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable diagnostic printouts for event 3 .\n
 *  "Cfg.SetEventEnablePrintouts(3,1)" //Command string to ecmcCmdParser.c\n
 */
int setEventEnablePrintouts(int indexEvent,
                            int enable);

/** \brief Force trigger event.\n
 *
 * Any subscriber functions to an event will be executed.\n
 *
 * \param[in] indexEvent Index of event to address.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Force trigger event 1.\n
 *  "Cfg.TriggerEvent(1)" //Command string to ecmcCmdParser.c\n
 */
int triggerEvent(int indexEvent);

/** \brief Arm event.\n
 *
 * Manually execute arm sequence, see command setEventEnableArmSequence().\n
 *
 * \param[in] indexEvent Index of event to address.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Run arm sequence for event 7.\n
 *  "Cfg.ArmEvent(7)" //Command string to ecmcCmdParser.c\n
 */
int armEvent(int indexEvent);

/** \brief Create a data storage object.
 *
 * The data storage object is a data buffer.
 *
 * Currently only the recorder object utilizes the data storage object.
 * However, it's foreseen that it will also be useful to store trajectory array
 * data in\n.
 *
 * \todo A method to copy the data to an EPICS waveform needs to be
 * implemented.\n
 *
 * \param[in] index Index of data storage object to create.\n
 * \param[in] elements Size of data buffer.\n
 * \param[in] bufferType Data buffer type.\n
 *  bufferType = 0: Normal  buffer (fill from beginning stop when full).\n
 *  bufferType = 1: Ring buffer (fill from beginning start over when full).\n
 *  bufferType = 2: FIFO buffer (fill from end. old values shifted out).\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Create a LIFO data storage object with 1000 elements at
 * index 1.\n
 *  "Cfg.CreateStorage(1,1000,0)" //Command string to ecmcCmdParser.c\n
 */
int createDataStorage(int index,
                      int elements,
                      int bufferType);

/** \brief Clear data storage buffer.
 *
 *  Erases all data within a data storage object.
 *
 * \param[in] index Index of data storage object to clear.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Clear data storage object with index 4.\n
 *  "Cfg.ClearStorage(4)" //Command string to ecmcCmdParser.c\n
 */
int clearStorage(int indexStorage);

/** \brief Get current index of data in storage buffer.\n
 *
 * \param[in] index Index of data storage object to clear.\n
 * \param[out] index Current data element index.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get current data element index for data storage object
 * with index 4.\n
 *  "GetStorageDataIndex(4)" //Command string to ecmcCmdParser.c\n
 */
int getStorageDataIndex(int  indexStorage,
                        int *index);

/** \brief Enable diagnostic printouts from data storage object.\n
 *
 * \param[in] indexStroage Index of data storage object to address.\n
 * \param[in] enable Enable diagnostic printouts.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable diagnostic printouts for data storage object 3 .\n
 *  "Cfg.SetStorageEnablePrintouts(3,1)" //Command string to ecmcCmdParser.c\n
 */
int setStorageEnablePrintouts(int indexStorage,
                              int enable);

/** \brief Print contents of buffer.\n
 *
 * \param[in] indexStroage Index of data storage object to address.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Print contents of data storage object 3 .\n
 *  "Cfg.PrintDataStorage(3)" //Command string to ecmcCmdParser.c\n
 */
int printStorageBuffer(int indexStorage);

/** \brief Reads contents of storage buffer.\n
 *
 * \param[in] indexStorage Index of data storage object to address.\n
 * \param[out] data Pointer to data.\n
 * \param[out] size Number of data elements.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Read contents of data storage object 3 .\n
 *  "ReadDataStorage(3)" //Command string to ecmcCmdParser.c\n
 */
int readStorageBuffer(int      indexStorage,
                      double **data,
                      int     *size);

/** \brief Writes data to storage buffer.\n
 *
 * \param[in] indexStorage Index of data storage object to address.\n
 * \param[in] data Pointer to data.\n
 * \param[in] size Number of data elements to write.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Write contents of data storage object 3 .\n
 *  "WriteDataStorage(3)=0,0,0,0,0,0...." //Command string to ecmcCmdParser.c\n
 */
int writeStorageBuffer(int     indexStorage,
                       double *data,
                       int     size);

/** \brief Appends data to the end of storage buffer.\n
 *
 * \param[in] indexStorage Index of data storage object to address.\n
 * \param[in] data Pointer to data.\n
 * \param[in] size Number of data elements to write.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Append data to data storage object 3 .\n
 *  "AppendDataStorage(3)=0,0,0,0,0,0....." //Command string to ecmcCmdParser.c\n
 */
int appendStorageBuffer(int     indexStorage,
                        double *data,
                        int     size);

/** \brief Set current data index of storage buffer.\n
 *
 * This function can be used to set teh current index of a data
 * storage buffer. A subsequent append of data will start at this
 * position index in the buffer array.
 *
 * \param[in] indexStorage Index of data storage object to address.\n
 * \param[in] position Position index of data.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Set current index of data storage 0 to 10.\n
 *  "Cfg.SetDataStorageCurrentDataIndex(0,10)" //Command string to ecmcCmdParser.c\n
 */
int setDataStorageCurrentDataIndex(int indexStorage,
                                   int position);

/** \brief Create recorder object.
 *
 * The recorder object stores data, from an EtherCAT entry, in a data storage
 * object. The recording of a data point can be triggered by an event or by
 * the command triggerRecorder().\n
 *
 * An EtherCAT data entry needs to be linked to the object with the command
 * linkEcEntryToRecorder().\n
 * The data recorder object needs a data storage object to store data in. The
 * data storage object needs to be linked to the recorder object with the
 * command linkStorageToRecorder().\n
 *
 * \param[in] index Index of recorder object to create.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Create a recorder object at index 1.\n
 * "Cfg.CreateRecorder(1)" //Command string to ecmcCmdParser.c\n
 */
int createRecorder(int indexRecorder);

/** \brief Link storage object to recorder object.
 *
 * \param[in] indexStorage Index of storage object.\n
 * \param[in] indexRecorder Index of recorder object.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Link storage object 5 to recorder object 3.\n
 * "Cfg.LinkStorageToRecorder(5,3)" //Command string to ecmcCmdParser.c\n
 */
int linkStorageToRecorder(int indexStorage,
                          int indexRecorder);

/** \brief Links an EtherCAT entry to a recorder object. \n
 *
 *  \param[in] indexRecorder Index of recorder object to link to.\n
 *  \param[in] recorderEntryIndex Index of recorder objects entry list.\n
 *    recorderEntryIndex = 0: data input.\n
 *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
 *    slaveBusPosition = -1: Used to address the simulation slave. Only two
 *                           entries are configured, "ZERO" with default
 *                           value 0 and "ONE" with default value 1.\n
 *    slaveBusPosition = 0..65535: Addressing of normal EtherCAT slaves.\n
 *  \param[in] entryIdString String for addressing purpose (see command
 *                      "Cfg.EcAddEntryComplete() for more information").\n
 *  \param[in] entryBitIndex Bit index of EtherCAT entry to use.\n
 *    entryBitIndex = -1: All bits of the entry will be used.\n
 *    entryBitIndex = 0..64: Only the selected bit will be used.\n
 *
 *
 * \return 0 if success or otherwise an error code.\n
 *
 *  \note Example 1: Link bit 0 of an EtherCAT entry configured as "INPUT_0"
 *  in slave 1 as data for recorder object 7.\n
 *  "Cfg.LinkEcEntryToRecorder(7,0,1,"INPUT_0",0)" //Command string to ecmcCmdParser.c\n
 *
 * \todo This function have not consistent parameter order with the other
 *  link functions as "linkEcEntryToAxisMon".\n
 */
int linkEcEntryToRecorder(int   indexRecorder,
                          int   recorderEntryIndex,
                          int   slaveBusPosition,
                          char *entryIDString,
                          int   bitIndex);

/** \brief Links an axis data source to a recorder object. \n
 *
 *  \param[in] indexRecorder Index of recorder object to link to.\n
 *  \param[in] axisIndex Index of axis to get data from.\n
 *  \param[in] dataToStore data to record from axis object.\n
 *    dataToStore = 0 : No data choosen.\n
 *    dataToStore = 1 : Position Setpoint (from trajectory generator).\n
 *    dataToStore = 2 : Position Actual (scaled).\n
 *    dataToStore = 3 : Position Error.\n
 *    dataToStore = 4 : Position Target.\n
 *    dataToStore = 5 : Controller Error.\n
 *    dataToStore = 6 : Controller Output.\n
 *    dataToStore = 7 : Velocity Setpoint.\n
 *    dataToStore = 8 : Velocity Actual.\n
 *    dataToStore = 9 : Velocity Setpoint Raw.\n
 *    dataToStore = 10: Velocity Setpoint Feed Forward Raw.\n
 *    dataToStore = 11: Error Code.\n
 *    dataToStore = 12: Enable (command).\n
 *    dataToStore = 13: Enabled (status).\n
 *    dataToStore = 14: Execute (command).\n
 *    dataToStore = 15: Busy (status).\n
 *    dataToStore = 16: Sequence state.\n
 *    dataToStore = 17: At Target (status).\n
 *    dataToStore = 18: Interlock type.\n
 *    dataToStore = 19: Limit Switch forward.\n
 *    dataToStore = 20: Limit Switch backward.\n
 *    dataToStore = 21: Home switch.\n
 *    dataToStore = 22: Command.\n
 *    dataToStore = 23: Command Data (cmdData).\n
 *    dataToStore = 24: Trajectory setpoint source.\n
 *          0  = Internal Source.\n
 *          >0 = External Source.\n
 *    dataToStore = 25: Encoder setpoint source.\n
 *          0  = Internal Source.\n
 *          >0 = External Source.\n
 *    dataToStore = 26: Axis Id.\n
 *    dataToStore = 27: Cycle counter.\n
 *    dataToStore = 28: Position Raw.\n
 *    dataToStore = 29: Encoder homed.\n

 * \return 0 if success or otherwise an error code.\n
 *
 *  \note Example 1: Link Actual position of axis 4 to recorder 1:.\n
 *  in slave 1 as data for recorder object 7.\n
 *  "Cfg.LinkAxisDataToRecorder(1,4,2)" //Command string to ecmcCmdParser.c\n
 */
int linkAxisDataToRecorder(int indexRecorder,
                           int axisIndex,
                           int dataToStore);

/** \brief Enable recorder.\n
 *
 * Recording of data is only active when the enable bit is high.\n
 *
 * \param[in] indexRecorder Index of recorder to address.\n
 * \param[in] enable Enable.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Start data recording of recorder object 4.\n
 *  "Cfg.SetRecorderEnable(4,1)" //Command string to ecmcCmdParser.c\n
 */
int setRecorderEnable(int indexRecorder,
                      int enable);

/** \brief Get recorder enabled.\n
 *
 * Recording of data is only active when the enable bit is high.\n
 *
 * \param[in] indexRecorder Index of recorder to address.\n
 * \param[out] enabled Enabled.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Get recorder object 4 enabled.\n
 *  "GetRecorderEnabled(4)" //Command string to ecmcCmdParser.c\n
 */
int getRecorderEnabled(int  indexRecorder,
                       int *enabled);

/** \brief Enable diagnostic printouts from recorder object.\n
 *
 * \param[in] indexRecorder Index of recorder object to address.\n
 * \param[in] enable Enable diagnostic printouts.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable diagnostic printouts for recorder object 3 .\n
 *  "Cfg.SetRecorderEnablePrintouts(3,1)" //Command string to ecmcCmdParser.c\n
 */
int setRecorderEnablePrintouts(int indexRecorder,
                               int enable);

/** \brief Link recorder object to event object.\n
 *
 * \param[in] indexRecorder Index of recorder object to address.\n
 * \param[in] indexEvent Index of event object to address.\n
 * \param[in] consumerIndex Event consumer index (one event can have a
 * list with consumers).\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Link recorder object 3 with event object 4, event consumer index 0.\n
 *  "Cfg.LinkRecorderToEvent(3,4,0)" //Command string to ecmcCmdParser.c\n
 */
int linkRecorderToEvent(int indexRecorder,
                        int indexEvent,
                        int consumerIndex);

/** \brief Force trigger recorder.\n
 *
 * The recorder object will store data in the linked data storage object.\n
 *
 * \param[in] indexRecorder Index of recorder to address.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Force trigger recorder 1.\n
 *  "Cfg.TriggerRecorder(1)" //Command string to ecmcCmdParser.c\n
 */
int triggerRecorder(int indexRecorder);

/** \brief Create a command list object.
 *
 * The command list object consists of a list of commands that can be executed.
 * Currently any command can be added to the command list. The execution of the
 * command list can be triggered by an event or by the command
 * triggerCommandList().\n
 *
 * \note If the command list is triggered by a hardware event. The command
 * list will be executed from the realtime thread. In this case special
 * consideration needs to be taken so that the jitter of realtime thread is not
 * worsen. Normal motion commands are normally no issue to use but commands
 * that creates new axis or takes long time to return should be avoided.\n
 *
 * \param[in] index Index of command list object to create.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Create a command list object at index 1.\n
 * "Cfg.CreateCommandList(1)" //Command string to ecmcCmdParser.c\n
 */
int createCommandList(int indexCommandList);

/** \brief Link command list object to event object.\n
 *
 * \param[in] indexCommandList Index of command list object to address.\n
 * \param[in] indexEvent Index of event object to address.\n
 * \param[in] consumerIndex Event consumer index (one event can have a
 * list with consumers).\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Link coommand list object 3 with event object 4, event consumer index 0.\n
 *  "Cfg.LinkCommandListToEvent(3,4,0)" //Command string to ecmcCmdParser.c\n
 */
int linkCommandListToEvent(int indexCommandList,
                           int indexEvent,
                           int consumerIndex);

/** \brief Enable command list.\n
 *
 * Command list will only be executed when the enable bit is high.\n
 *
 * \param[in] indexCommandList Index of command list to address.\n
 * \param[in] enable Enable.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: "Enable" command list execution for object 4.\n
 *  "Cfg.SetCommandListEnable(4,1)" //Command string to ecmcCmdParser.c\n
 */
int setCommandListEnable(int indexCommandList,
                         int enable);

/** \brief Enable diagnostic printouts from command list object.\n
 *
 * \param[in] indexCommandList Index of command list object to address.\n
 * \param[in] enable Enable diagnostic printouts.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable diagnostic printouts for command list object 3 .\n
 *  "Cfg.SetCommandListEnablePrintouts(3,1)" //Command string to ecmcCmdParser.c\n
 */
int setCommandListEnablePrintouts(int indexCommandList,
                                  int enable);

/** \brief Add command to command list.\n
 *
 * \param[in] indexCommandList Index of command list object to address.\n
 * \param[in] expr Command string.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Add "Main.M1.bExecute=1" to command list 1.\n
 *  "Cfg.AddCommandToCommandList(1)=Main.M1.bExecute=1" //Command string to
 *  ecmcCmdParser.c\n
 */
int addCommandListCommand(int   indexCommandList,
                          char *expr);

/** \brief Force trigger command list.\n
 *
 * All commands in the command list object will be executed.\n
 *
 * \param[in] indexCommandList Index of command list to address.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Force trigger command list 1.\n
 *  "Cfg.TriggerCommandList(1)" //Command string to ecmcCmdParser.c\n
 */
int triggerCommandList(int indexCommandList);

/** \brief Create SHM Object.\n
 *
 * Creates a shared memory object for master 2 master communication
 *
 * \return 0 if success or otherwise an error code.\n
 *
 */
int createShm();

# ifdef __cplusplus
}
# endif  // ifdef __cplusplus

#endif  /* ECMC_MISC_H_ */