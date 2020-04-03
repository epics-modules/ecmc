/*************************************************************************\
* Copyright (c) 2019 European Spallation Source ERIC
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
*
*  ecmcEthercat.h
*
*  Created on: Jan 10, 2019
*      Author: anderssandstrom
*
\*************************************************************************/

#ifndef ECMC_ETHERCAT_H_
#define ECMC_ETHERCAT_H_

# ifdef __cplusplus
extern "C" {
# endif  // ifdef __cplusplus

/** \brief Selects EtherCAT master to use.\n
 *
 *  \param[in] masterIndex EtherCAT master index.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Select /dev/EtherCAT0.\n
 *  "Cfg.EcSetMaster(0)" //Command string to ecmcCmdParser.c\n
 */
int ecSetMaster(int masterIndex);

/** \brief  Retry configuring slaves for an selected EtherCAT master.\n
 *
 * Via this method, the application can tell the master to bring all slaves to
 * OP state. In general, this is not necessary, because it is automatically
 * done by the master. But with special slaves, that can be reconfigured by
 * the vendor during runtime, it can be useful.
 *
 * \note  masterIndex is not used. Currently only one master is supported and
 * this master will be reset. The masterIndex is for future use.
 *
 *  \param[in] masterIndex EtherCAT master index.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Reset master. Master index is NOT currently supported.\n
 *  "Cfg.EcResetMaster(10)" //Command string to ecmcCmdParser.c\n
 */
int ecResetMaster(int masterIndex);

/** \brief Adds an EtherCAT slave to the hardware configuration.\n
 *
 * Each added slave will be assigned an additional index which will be zero for
 * the first successfully added slave and then incremented for each successful
 * call to "Cfg.EcAddSlave()".\n
 *
 * NOTE: if an complete entry needs to be configured the command
 * "Cfg.EcAddEntryComplete()" should be used. This command will add slave,
 * sync. manger, pdo and entry if needed.\n
 *
 *  \param alias Alias of slave. Set to zero to disable.\n
 *  \param slaveBusPosition Position of the EtherCAT slave on the bus.\n
 *    slaveBusPosition = -1: Used to address the simulation slave. Only two
 *                           entries are configured, "ZERO" with default
 *                           value 0 and "ONE" with default value 1.\n
 *    slaveBusPosition = 0..65535: Addressing of normal EtherCAT slaves.\n
 *  \param vendorId Identification value for slave vendor.\n
 *    vendorId = 0x2: Beckhoff.\n
 *    vendorId = 0x48554B: Kendrion Kuhnke Automation GmbH.\n
 *  \param productCode Product identification code.\n
 *    productCode=0x13ed3052: EL5101 incremental encoder input.\n
 *
 * \note All configuration data can be found in the documentaion of
 * the slave, in the ESI slave description file or by using the etherlab
 * (www.etherlab.org) ethercat tool.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Add a EL5101 Beckhoff slave at slave position 1.\n
 *  "Cfg.EcAddSlave(0,1,0x2,0x13ed3052)" //Command string to ecmcCmdParser.c\n
 */
int ecAddSlave(uint16_t alias,
               uint16_t position,
               uint32_t vendorId,
               uint32_t productCode);

/// obsolete command. Use ecAddEntryComplete() command instead.\n
int ecAddSyncManager(int     slaveIndex,
                     int     direction,
                     uint8_t syncMangerIndex);

/// obsolete command. Use ecAddEntryComplete() command instead.\n
int ecAddPdo(int      slaveIndex,
             int      syncManager,
             uint16_t pdoIndex);

/** \breif Adds an EtherCAT slave to the hardware configuration.\n
 *
 * Each added slave will be assigned an additional index which will be zero for
 * the first successfully added slave and then incremented for each successful
 * call to "Cfg.EcAddSlave()".\n
 *
 *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
 *    slaveBusPosition = -1: Used to address the simulation slave. Only two
 *                           entries are configured, "ZERO" with default
 *                           value 0 and "ONE" with default value 1.\n
 *    slaveBusPosition = 0..65535: Addressing of normal EtherCAT slaves.\n
 *  \param[in] vendorId Identification value for slave vendor.\n
 *    vendorId = 0x2: Beckhoff.\n
 *    vendorId = 0x48554B: Kendrion Kuhnke Automation GmbH.\n
 *  \param[in] productCode Product identification code.\n
 *    productCode=0x13ed3052: EL5101 incremental encoder input.\n
 *  \param[in] direction Data transfer direction..\n
 *    direction  = 1:  Output (from master).\n
 *    direction  = 2:  Input (to master).\n
 *  \param[in] syncMangerIndex Index of sync manager.
 *  \param[in] pdoIndex Index of process data object. Needs to be entered
 *                           in hex format.\n
 *  \param[in] entryIndex Index of process data object entry. Needs to be
 *                           entered in hex format.\n
 *  \param[in] entrySubIndex Index of process data object sub entry.
 *                           Needs to be entered in hex format.\n
 *  \param[in] bits Bit count.\n
 *  \param[in] entryIDString Identification string used for addressing the
 *                           entry.\n
 *  \param[in] signedValue 1 if value is of signed type otherwise 0
 *
 *
 * \note All configuration data can be found in the documentation of
 * the slave, in the ESI slave description file or by using the etherlab
 * (www.etherlab.org) ethercat tool.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Add an EtherCAT entry for the actual position of an EL5101
 * incremental encoder card.\n
 * "Cfg.EcAddEntryComplete(2,0x2,0x13ed3052,2,3,0x1a03,0x6010,0x10,16,POSITION)"
 * //Command string to ecmcCmdParser.c\n
 *
 * \note Example: Add an EtherCAT entry for the velocity setpoint of an EL7037
 * stepper drive card.\n
 * "Cfg.EcAddEntryComplete(7,0x2,0x1b7d3052,1,2,0x1604,0x7010,0x21,16,
 * VELOCITY_SETPOINT)" //Command string to ecmcCmdParser.c\n
 */
int ecAddEntryComplete(
  uint16_t slaveBusPosition,
  uint32_t vendorId,
  uint32_t productCode,
  int      direction,
  uint8_t  syncMangerIndex,
  uint16_t pdoIndex,
  uint16_t entryIndex,
  uint8_t  entrySubIndex,
  uint8_t  bits,
  char    *entryIDString,
  int      signedValue);

/** \brief Adds an EtherCAT slave to the hardware configuration.\n
 *
 * Each added slave will be assigned an additional index which will be zero for
 * the first successfully added slave and then incremented for each successful
 * call to "Cfg.EcAddSlave()".\n
 *
 *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
 *    slaveBusPosition = -1: Used to address the simulation slave. Only two
 *                           entries are configured, "ZERO" with default
 *                           value 0 and "ONE" with default value 1.\n
 *    slaveBusPosition = 0..65535: Addressing of normal EtherCAT slaves.\n
 *  \param[in] vendorId Identification value for slave vendor.\n
 *    vendorId = 0x2: Beckhoff.\n
 *    vendorId = 0x48554B: Kendrion Kuhnke Automation GmbH.\n
 *  \param[in] productCode Product identification code.\n
 *    productCode=0x13ed3052: EL5101 incremental encoder input.\n
 *  \param[in] direction Data transfer direction..\n
 *    direction  = 1:  Output (from master).\n
 *    direction  = 2:  Input (to master).\n
 *  \param[in] syncMangerIndex Index of sync manager.
 *  \param[in] pdoIndex Index of process data object. Needs to be entered
 *                           in hex format.\n
 *  \param[in] entryIndex Index of process data object entry. Needs to be
 *                           entered in hex format.\n
 *  \param[in] entrySubIndex Index of process data object sub entry.
 *                           Needs to be entered in hex format.\n
 *  \param[in] dataType DataType of ethercat data:\n
 *                      B1:  1-bit\n
 *                      B2:  2-bits (lsb)\n
 *                      B3:  3-bits (lsb)\n
 *                      B4:  3-bits (lsb)\n
 *                      U8:  Unsigned 8-bit\n
 *                      S8:  Signed 8-bit\n
 *                      U16: Unsigned 16-bit\n
 *                      S16: Signed 16-bit\n
 *                      U32: Unsigned 32-bit\n
 *                      S32: Signed 32-bit\n
 *                      U64: Unsigned 64-bit\n
 *                      S64: Signed 64-bit\n
 *                      F32: Real 32-bit\n 
 *                      F64: Double 64-bit\n
 * 
 *  \param[in] entryIDString Identification string used for addressing the
 *                           entry.\n
 *  \param[in] updateInRT    1 if value should be updated in realtime 0\n
 *                           normally set to zero for entries that are\n
 *                           covered in memmaps.\n
 *
 * \note All configuration data can be found in the documentation of
 * the slave, in the ESI slave description file or by using the etherlab
 * (www.etherlab.org) ethercat tool.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Add an EtherCAT entry for the actual position of an EL5101
 * incremental encoder card.\n
 * "Cfg.EcAddEntry(2,0x2,0x13ed3052,2,3,0x1a03,0x6010,0x10,U16,POSITION,1)"
 * //Command string to ecmcCmdParser.c\n
 *
 * \note Example: Add an EtherCAT entry for the velocity setpoint of an EL7037
 * stepper drive card.\n
 * "Cfg.EcAddEntry(7,0x2,0x1b7d3052,1,2,0x1604,0x7010,0x21,S16,
 * VELOCITY_SETPOINT,1)" //Command string to ecmcCmdParser.c\n
 */
int ecAddEntry(
  uint16_t slaveBusPosition,
  uint32_t vendorId,
  uint32_t productCode,
  int      direction,
  uint8_t  syncMangerIndex,
  uint16_t pdoIndex,
  uint16_t entryIndex,
  uint8_t  entrySubIndex,
  char    *datatype,  
  char    *entryIDString,
  int      updateInRealTime);

/** \brief Adds a memory map object to access data directly from EtherCAT
 *   domain. This is the preferred syntax.\n
 *
 *  The start of the memory map is addressed by a previously configured
 *  EtherCAT entry and a size.
 *
 *  \param[in] ecPath   Identification string of the start EtherCAT
 *   *                  entry (example "ec0.s1.AI_1").\n
 *  \param[in] byteSize Size of memory map objects (size to access).\n
 *  \param[in] direction Data transfer direction..\n
 *    direction  = 1:  Output (from master).\n
 *    direction  = 2:  Input (to master).\n
 *  \param[in] dataType DataType of ethercat data:\n
 *                      B1:  1-bit\n
 *                      B2:  2-bits (lsb)\n
 *                      B3:  3-bits (lsb)\n
 *                      B4:  3-bits (lsb)\n
 *                      U8:  Unsigned 8-bit\n
 *                      S8:  Signed 8-bit\n
 *                      U16: Unsigned 16-bit\n
 *                      S16: Signed 16-bit\n
 *                      U32: Unsigned 32-bit\n
 *                      S32: Signed 32-bit\n
 *                      U64: Unsigned 64-bit\n
 *                      S64: Signed 64-bit\n
 *                      F32: Real 32-bit\n 
 *                      F64: Double 64-bit\n
 * 
 *  \param[in] memMapIdString Identification string used for addressing the
 object.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Add an EtherCAT input memory map of size 200 bytes starting at
 * entry "AI1" on slave 10. Name the memory map WAVEFORM. Type
 * argument is excluded in\n
 * "Cfg.EcAddMemMapDT(10,AI1,200,2,ec0.mm.WAVEFORM)" //Command string to ecmcCmdParser.c\n
 */
int ecAddMemMapDT(
  char    *ecPath,  
  size_t   byteSize,
  int      direction,
  char    *dataType,
  char    *memMapIDString);

/** \brief Adds a memory map object to access data directly from EtherCAT\n
 *   domain. Support for this syntax might be dropped in newer releases.\n 
 *   Please use new syntax.\n
 *
 *  The start of the memory map is addressed by a previously configured
 *  EtherCAT entry and a size.
 *
 *  \param[in] startEntryBusPosition Position of the EtherCAT slave on the bus
 *                                   where the start entry is configured.\n
 *    startEntryBusPosition = 0..65535: Addressing of normal EtherCAT slaves.\n
 *  \param[in] startEntryIDString I Identification string of the start EtherCAT
 *   *                           entry.\n
 *  \param[in] byteSize Size of memory map objects (size to access).\n
 *  \param[in] direction Data transfer direction..\n
 *    direction  = 1:  Output (from master).\n
 *    direction  = 2:  Input (to master).\n
 * 
 *  \param[in] entryIDString Identification string used for addressing the
 object.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Add an EtherCAT input memory map of size 200 bytes starting at
 * entry "AI1" on slave 10. Name the memory map WAVEFORM. Type
 * argument is excluded in\n
 * "Cfg.EcAddMemMap(10,AI1,200,2,ec0.mm.WAVEFORM)" //Command string to ecmcCmdParser.c\n
 */
int ecAddMemMap(
  uint16_t startEntryBusPosition,
  char    *startEntryIDString,
  size_t   byteSize,
  int      direction,
  char    *memMapIDString);

/** \brief Configure slave DC clock.\n
 *
 *
 *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
 *    slaveBusPosition = 0..65535: Addressing of EtherCAT slaves.\n*
 *  \param[in] assignActivate A* Each added slave will be assigned an additional index which will be zero for
 * the first successfully added slave and then incremented for each successful
 * call to "Cfg.EcAddSlave()".\n
 ssign active word. This information can be found
 *                           in the ESI slave description file. Needs to be
 *                           entered in hex format.\n
 *  \param[in] sync0Cycle Cycle time in ns.\n
 *  \param[in] sync0Shift Phase shift in ns.\n
 *  \param[in] sync1Cycle Cycle time in ns.\n
 *  \param[in] sync1Shift Not used.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Configure the DC clock of an EL5101 incremental encoder
 * input card: slaveBusPosition=1, assignActivate=0x320, sync0Cycle=1000000
 * (1kHz), sync0Shift=10, sync1Cycle=1000000 (1kHz).\n
 * "Cfg.EcSlaveConfigDC(1,0x320,1000000,10,1000000,0)" //Command string to
 * ecmcCmdParser.c\n
 */
int ecSlaveConfigDC(
  int      slaveBusPosition,
  uint16_t assignActivate,   /**< AssignActivate word. */
  uint32_t sync0Cycle,   /**< SYNC0 cycle time [ns]. */
  int32_t  sync0Shift,  /**< SYNC0 shift time [ns]. */
  uint32_t sync1Cycle,   /**< SYNC1 cycle time [ns]. */
  int32_t  sync1Shift  /**< SYNC1 shift time [ns]. */);

/** \brief Select EtherCAT reference clock.\n
 *
 *  \param[in] masterIndex Index of master, see command ecSetMaster().\n
 *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.
 *                              Note: the slave needs to be equipped
 *                              with a DC.\n
 *    slaveBusPosition = 0..65535: Addressing of EtherCAT slaves.\n*
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Select slave 3 as reference clock for master 0.\n
 * "Cfg.EcSelectReferenceDC(0,3)" //Command string to ecmcCmdParser.c\n
 */
int ecSelectReferenceDC(int masterIndex,
                        int slaveBusPosition);

/** \brief Adds a Service Data Object for writing.
 *
 * Adds a Service Data Object for writing to the sdo registers of EtherCAT
 * slaves. An sdo object will be added to the hardware configuration.
 * The writing will occur when the hardware configuration is applied.\n
 *
 * \note This command can only be used in configuration mode.\n
 *
 *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
 *    slaveBusPosition = 0..65535: Addressing of EtherCAT slaves.\n
 *  \param[in] sdoIndex Index of service data object. Needs to be
 *                           entered in hex format.\n
 *  \param[in] sdoSubIndex Sub index of service data object .
 *                           Needs to be entered in hex format.\n
 *  \param[in] value Value to write.\n
 *  \param[in] byteSize Byte count to write.\n
 *
 * \note All configuration data can be found in the documentation of
 * the slave, in the ESI slave description file or by using the etherlab
 * (www.etherlab.org) ethercat tool.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Write 1A (1000mA) to maximum current of the EL7037 stepper drive card
 * on slave position 2.\n
 * "Cfg.EcAddSdo(2,0x8010,0x1,1000,2)" //Command string to ecmcCmdParser.c\n
 */
int ecAddSdo(uint16_t slaveBusPosition,
             uint16_t sdoIndex,
             uint8_t  sdoSubIndex,
             uint32_t value,
             int      byteSize);

/** \brief Adds a Service Data Object for writing.
 *
 * Adds a Service Data Object for writing to the sdo registers of EtherCAT
 * slaves. An sdo object will be added to the hardware configuration.
 * The writing will occur when the hardware configuration is applied.\n
 *
 * \note This command can only be used in configuration mode.\n
 *
 *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
 *    slaveBusPosition = 0..65535: Addressing of EtherCAT slaves.\n
 *  \param[in] sdoIndex Index of service data object. Needs to be
 *                           entered in hex format.\n
 *  \param[in] valueBuffer Values to be written as hex string, each byte separted with space.\n
 *  \param[in] byteSize Byte count to write.\n
 *
 * \note All configuration data can be found in the documentation of
 * the slave, in the ESI slave description file or by using the etherlab
 * (www.etherlab.org) ethercat tool.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Write a hex string of data to slave position 2.\n
 * "Cfg.EcAddSdoComplete(2,0x8010,0A FF CA 01 25 F1,6)" //Command string to ecmcCmdParser.c\n
 */
int ecAddSdoComplete(uint16_t    slaveBusPosition,
                     uint16_t    sdoIndex,                     
                     const char* valueBuffer,
                     int         byteSize);

/** \brief Adds a Service Data Object for writing.
 *
 * Adds a Service Data Object for writing to the sdo registers of EtherCAT
 * slaves. An sdo object will be added to the hardware configuration.
 * The writing will occur when the hardware configuration is applied.\n
 *
 * \note This command can only be used in configuration mode.\n
 *
 *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
 *    slaveBusPosition = 0..65535: Addressing of EtherCAT slaves.\n
 *  \param[in] sdoIndex Index of service data object. Needs to be
 *                           entered in hex format.\n
 *  \param[in] sdoSubIndex Sub index of service data object. Needs to be
 *                         entered in hex format.\n
 *  \param[in] valueBuffer Values to be written as hex string, each byte separted with space.\n
 *  \param[in] byteSize Byte count to write.\n
 *
 * \note All configuration data can be found in the documentation of
 * the slave, in the ESI slave description file or by using the etherlab
 * (www.etherlab.org) ethercat tool.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Write a hex string of data to slave position 2.\n
 * "Cfg.EcAddSdoBuffer(2,0x8010,0x1,0A FF CA 01 25 F1,6)" //Command string to ecmcCmdParser.c\n
 */
int ecAddSdoBuffer(uint16_t    slavePosition,
                   uint16_t    sdoIndex,
                   uint8_t     sdoSubIndex,
                   const char* valueBuffer,
                   int         byteSize);

/** \brief Write to a Service Data Object.
 *
 * Writing will occur directly when the command is issued.\n
 *
 * \note This command can only be used in configuration mode.\n
 *
 *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
 *    slaveBusPosition = 0..65535: Addressing of EtherCAT slaves.\n
 *  \param[in] sdoIndex Index of service data object. Needs to be
 *                           entered in hex format.\n
 *  \param[in] sdoSubIndex Sub index of service data object .
 *                           Needs to be entered in hex format.\n
 *  \param[in] value Value to write.\n
 *  \param[in] byteSize Byte count to write.\n
 *
 * \note All configuration data can be found in the documentation of
 * the slave, in the ESI slave description file or by using the etherlab
 * (www.etherlab.org) ethercat tool.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Write 1A (1000mA) to maximum current of the EL7037 stepper drive card
 * on slave position 2.\n
 * "Cfg.EcWriteSdo(2,0x8010,0x1,1000,2)" //Command string to ecmcCmdParser.c\n
 */
int ecWriteSdo(uint16_t slavePosition,
               uint16_t sdoIndex,
               uint8_t  sdoSubIndex,
               uint32_t value,
               int      byteSize);

/** \brief Write to a Service Data Object.
 *
 * Note: same  as "ecWriteSdo(uint16_t slavePposition,uint16_t sdoIndex,uint8_t sdoSubIndex,
 * uint32_t value,int byteSize)" but without subindex. Complete SDO access will be used.
 *
 */
/*int ecWriteSdoComplete(uint16_t slavePosition,
                       uint16_t sdoIndex,
                       uint32_t value,
                       int      byteSize);*/

/** \brief Read a Service Data Object.
 *
 * Read will occur directly when the command is issued.\n
 *
 * \note This command can only be used in configuration mode.\n
 *
 *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
 *    slaveBusPosition = 0..65535: Addressing of EtherCAT slaves.\n
 *  \param[in] sdoIndex Index of service data object. Needs to be
 *                           entered in hex format.\n
 *  \param[in] sdoSubIndex Sub index of service data object .
 *                           Needs to be entered in hex format.\n
 *  \param[in] byteSize Byte count to read.\n
 *  \param[out] data      Read data.\n
 *
 * \note All configuration data can be found in the documentation of
 * the slave, in the ESI slave description file or by using the etherlab
 * (www.etherlab.org) ethercat tool.\n
 *
 * \return 0 if success other wise an error code.\n
 *
 * \note Example: Read maximum current setting of the EL7037 stepper drive card
 * on slave position 2.\n
 * "Cfg.EcReadSdo(2,0x8010,0x1,2)" //Command string to ecmcCmdParser.c\n
 */
int ecReadSdo(uint16_t  slavePosition,
              uint16_t  sdoIndex,
              uint8_t   sdoSubIndex,
              int       byteSize,
              uint32_t *value);

/** \brief Verify a Service Data Object.
 *
 * Read will occur directly when the command is issued.\n
 *
 * \note This command can only be used in configuration mode.\n
 *
 *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
 *    slaveBusPosition = 0..65535: Addressing of EtherCAT slaves.\n
 *  \param[in] sdoIndex Index of service data object. Needs to be
 *                           entered in hex format.\n
 *  \param[in] sdoSubIndex Sub index of service data object .
 *                           Needs to be entered in hex format.\n
 *  \param[in] byteSize Byte count to read.\n
 *  \param[in] verValue      Expected value of SDO.\n
 *
 * \note All configuration data can be found in the documentation of
 * the slave, in the ESI slave description file or by using the etherlab
 * (www.etherlab.org) ethercat tool.\n
 *
 * \return 0 if success other wise an error code.\n
 *
 * \note Example: Verify maximum current setting of the EL7037 stepper drive card
 * on slave position 2 is set to 1000mA.\n
 * "Cfg.EcVerifySdo(2,0x8010,0x1,1000,2)" //Command string to ecmcCmdParser.c\n
 */
int ecVerifySdo(uint16_t  slavePosition,
                uint16_t  sdoIndex,
                uint8_t   sdoSubIndex,
                uint32_t  verValue,
                int       byteSize);

/** \brief Read SoE \n
 *
 * \note This command can only be used in configuration mode.\n
 *
 *  \param[in]  slaveBusPosition Position of the EtherCAT slave on the bus.\n
 *    slaveBusPosition = 0..65535: Addressing of EtherCAT slaves.\n
 *  \param[in]  driveNo   SoE drive number.\n
 *  \param[in]  idn       SoE IDN.\n
 *  \param[in]  byteSize  Byte count to read.\n
 *  \param[out] value     Pointer to read data.\n
 *
 * \note All configuration data can be found in the documentation of
 * the slave\n
 *
 * \return 0 if success other wise an error code.\n
 *
 * \note Example: Read data for drive 0
 * on slave position 2.\n
 * "Cfg.EcReadSoE(2,0,0x1000,2)" //Command string to ecmcCmdParser.c\n
 */
int ecReadSoE(uint16_t  slavePosition, /**< Slave position. */
              uint8_t   driveNo, /**< Drive number. */
              uint16_t  idn, /**< SoE IDN (see ecrt_slave_config_idn()). */
              size_t    byteSize, /**< Size of data to write. */
              uint8_t  *value /**< Pointer to data to write. */
             );
/** \brief Write SoE \n
 *
 * \note This command can only be used in configuration mode.\n
 *
 *  \param[in]  slaveBusPosition Position of the EtherCAT slave on the bus.\n
 *    slaveBusPosition = 0..65535: Addressing of EtherCAT slaves.\n
 *  \param[in]  driveNo   SoE drive number.\n
 *  \param[in]  idn       SoE IDN.\n
 *  \param[in]  byteSize  Byte count to write.\n
 *  \param[out] value     Pointer to data to write.\n
 *
 * \note All configuration data can be found in the documentation of
 * the slave\n
 *
 * \return 0 if success other wise an error code.\n
 *
 * \note Example: Read data for drive 0
 * on slave position 2.\n
 * "Cfg.EcWriteSoE(2,0,0x1000,2)" //Command string to ecmcCmdParser.c\n
 */
int ecWriteSoE(uint16_t  slavePosition, /**< Slave position. */
               uint8_t   driveNo, /**< Drive number. */
               uint16_t  idn, /**< SoE IDN (see ecrt_slave_config_idn()). */
               size_t    byteSize, /**< Size of data to write. */
               uint8_t  *value /**< Pointer to data to write. */
              );

/** \brief Configure Slave watch dog.\n
 *
 *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.
 *                              Note: the slave needs to be equipped
 *                              with a DC.\n
 *    slaveBusPosition = 0..65535: Addressing of EtherCAT slaves.\n
 *  \param[in] watchdogDivider  Number of 40 ns intervals. Used as a
 *                              base unit for all slave watchdogs. If set
 *                              to zero, the value is not written, so the
 *                              default is used.\n
 *  \param[in] watchdogIntervals Number of base intervals for process
 *                              data watchdog. If set to zero, the value
 *                              is not written, so the default is used.\n
 *
 * \note Example: Set watchdog times to 100,100 for slave at busposition 1.\n
 * "Cfg.EcSlaveConfigWatchDog(1,100,100)" //Command string to ecmcCmdParser.c\n
 *
 * \return 0 if success or otherwise an error code.\n
 */
int ecSlaveConfigWatchDog(int slaveBusPosition,
                          int watchdogDivider,
                          int watchdogIntervals);

/** \brief Apply hardware configuration to master.\n
 *
 * This command needs to be executed before entering runtime.\n
 *
 * \note This command can only be used in configuration mode.\n
 *
 *  \param[in] masterIndex Index of master, see command ecSetMaster().\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Apply hardware configuration to master 0 (/dev/EtherCAT0).\n
 * "Cfg.EcApplyConfig(0)" //Command string to ecmcCmdParser.c\n
 */
int ecApplyConfig(int masterIndex);

/** \brief Writes a value to an EtherCAT entry.\n
  *
  *  \param[in] slaveIndex Index of order of added slave (not bus position),
  *                        see command EcAddSlave()).\n
  *  \param[in] entryIndex Index of order of added entry (within slave).
  *  \param[in] value Value to be written.\n
  *
  * \note the slaveIndex and entryIndex can be read with the commands
  * readEcSlaveIndex() and readEcEntryIndexIDString().\n
  *
  * \return 0 if success or otherwise an error code.\n
  *
  * \note Example: Write a 0 to the 2:nd added entry (entryIndex=1) in the 4:th
  * added slave (slaveIndex=3).\n
  *  "WriteEcEntry(3,1,0)" //Command string to ecmcCmdParser.c\n
  */
int writeEcEntry(int      slaveIndex,
                 int      entryIndex,
                 uint64_t value);

/** \brief Writes a value to an EtherCAT entry addressed by slaveBusPosition
 * and entryIdString.\n
  *
  *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
  *    slaveBusPosition = -1: Used to address the simulation slave. Only two
  *                           entries are configured, "ZERO" with default
  *                           value 0 and "ONE" with default value 1.\n
  *    slaveBusPosition = 0..65535: Addressing of normal EtherCAT slaves.\n
  *  \param[in] entryIdString String for addressing purpose (see command
  *                      "Cfg.EcAddEntryComplete() for more information").\n
  *  \param[in] value Value to be written.\n
  *
  * Note: This command should not be used when realtime performance is needed
  * (also see "WriteECEntry()" command).\n
  *
  * \return 0 if success or otherwise an error code.\n
  *
  * \note Example: Write a 1 to a digital output configured as "OUTPUT_0" on slave 1\n
  *  "Cfg.WriteEcEntryIDString(1,OUTPUT_1,1)" //Command string to ecmcCmdParser.c\n
  */
int writeEcEntryIDString(int      slaveBusPosition,
                         char    *entryIdString,
                         uint64_t value);

/** \brief Read a value from an EtherCAT entry.\n
  *
  *  \param[in] slaveIndex Index of order of added slave (not bus position),
  *                        see command EcAddSlave()).\n
  *  \param[in] entryIndex Index of order of added entry (within slave).
  *  \param[out] value Read value (result).\n
  *
  * \note the slaveIndex and entryIndex can be read with the commands
  * readEcSlaveIndex() and readEcEntryIndexIDString().\n
  *
  * \return 0 if success or otherwise an error code.\n
  *
  * \note Example: Read the 2:nd added entry (entryIndex=1) in the 4:th added
  * slave (slaveIndex=3).\n
  * "ReadEcEntry(3,1)" //Command string to ecmcCmdParser.c\n
  */
int readEcEntry(int       slaveIndex,
                int       entryIndex,
                uint64_t *value);

/** \brief Read a value from an EtherCAT entry addressed by slaveBusPosition
 *   and entryIdString.\n
  *
  *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
  *    slaveBusPosition = -1: Used to address the simulation slave. Only two
  *                           entries are configured, "ZERO" with default
  *                           value 0 and "ONE" with default value 1.\n
  *    slaveBusPosition = 0..65535: Addressing of normal EtherCAT slaves.\n
  *  \param[in] entryIdString String for addressing purpose (see command
  *                      "Cfg.EcAddEntryComplete() for more information").\n
  *  \param[out] value Read value (result).\n
  *
  * Note: This command should not be used when realtime performance is needed
  * (also see "WriteECEntry()" command).\n
  *
  * \return 0 if success or otherwise an error code.\n
  *
  * \note Example: Read a digital input configured as "INPUT_0" on slave 1\n
  *  "ReadEcEntryIDString(1,INPUT_0)" //Command string to ecmcCmdParser.c\n
  */
int readEcEntryIDString(int       slavePosition,
                        char     *entryIDString,
                        uint64_t *value);

/** \brief Read the object Index of an entry addressed by slaveBusPosition
 *   and entryIdString.\n
  *
  * The first entry added in a slave will receive index 0, then for each added
  * entry, its index will be incremented by one. This index will be returned by
  * this function. This index is needed to address a certain slave with the
  * commands readEcEntry() and writeEcEntry() (for use in realtime, since
  * these functions address the slave and entry arrays directly via indices).\n
  *
  * \note This is not the same as the entryIndex in EcAddEntryComplete().\n
  *
  * \todo Change confusing naming of this function. entryIndex and index of entry is
  * easily confused.\n
  *
  *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
  *    slaveBusPosition = -1: Used to address the simulation slave. Only two
  *                           entries are configured, "ZERO" with default
  *                           value 0 and "ONE" with default value 1.\n
  *    slaveBusPosition = 0..65535: Addressing of normal EtherCAT slaves.\n
  *  \param[in] entryIdString String for addressing purpose (see command
  *                      "Cfg.EcAddEntryComplete() for more information").\n
  *  \param[out] value Entry index (the order is was added).\n
  *
  * \return 0 if success or otherwise an error code.\n
  *
  * \note Example: Read a the index of an entry configured as "INPUT_0" on slave 1\n
  *  "ReadEcEntryIndexIDString(1,INPUT_0)" //Command string to ecmcCmdParser.c\n
  */
int readEcEntryIndexIDString(int   slavePosition,
                             char *entryIDString,
                             int  *value);

/** \brief Read the object Index of an slave addressed by slaveBusPosition.\n
  *
  * The first slave added in will receive index 0, then for each added
  * slave, its index will be incremented by one. This index will be returned by
  * this function. This index is needed to address a certain slave with the
  * commands readEcEntry() and writeEcEntry() (for use in realtime, since
  * these functions address the slave and entry arrays directly via indices).\n
  *
  * \note slaveIndex and slaveBusPosition is not the same.\n
  *
  *  \param[in] slaveBusPosition Position of the EtherCAT slave on the bus.\n
  *    slaveBusPosition = -1: Used to address the simulation slave. Only two
  *                           entries are configured, "ZERO" with default
  *                           value 0 and "ONE" with default value 1.\n
  *    slaveBusPosition = 0..65535: Addressing of normal EtherCAT slaves.\n
  *
  * \return 0 if success or otherwise an error code.\n
  *
  * \note Example: Read the slave index of the slave with bus position 5.\n
  *  "ReadEcSlaveIndex(5)" //Command string to ecmcCmdParser.c\n
  */
int readEcSlaveIndex(int  slavePosition,
                     int *value);


/** \brief Read EtherCAT memory map object.
 *
 * Fast access of EtherCAT data from EPICS records is possible by linking an
 * EtherCAT memory map to an ASYN parameter. The memory map objects is most
 * usefull for acquire arrays of data (waveforms).This function is called by
 * the iocsh command"ecmcAsynPortDriverAddParameter()". For more information
 * see documentation of ecmcAsynPortDriverAddParameter(), ecAddMemMap(),
 * readEcMemMap() and ecSetEntryUpdateInRealtime().\n
 *
 *  \param[in] memMapIDString String for addressing ethercat entry:\n
 *             ec.mm.<memory map id>
 *  \param[out] *data Output data buffer.\n*
 *  \param[in]  bytesToRead Output data buffer size.\n*
 *  \param[out] bytesRead Bytes read.\n*
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: memMapIDString for an memory map called "AI_1_ARRAY":
 * "ec.mm.AI_1_ARRAY".\n
 * \note There's no ascii command in ecmcCmdParser.c for this method.\n
 */
int readEcMemMap(const char *memMapIDString,
                 uint8_t    *data,
                 size_t      bytesToRead,
                 size_t     *bytesRead);

/** \brief Set update in realtime bit for an entry
 *
 * If set to zero the entry will not be updated during realtime operation.\n
 * Useful when accessing data with memory maps instead covering many entries
 * like oversampling arrays (it's then unnecessary to update each entry in
 * array).\n
 *
 *  \param[in] slavePosition Position of the EtherCAT slave on the bus.\n
 *    slavePosition = -1: Used to address the simulation slave. Only two
 *                           entries are configured, "ZERO" with default
 *                           value 0 and "ONE" with default value 1.\n
 *    slaveBusPosition = 0..65535: Addressing of normal EtherCAT slaves.\n
 *  \param[in] entryIdString String for addressing purpose (see command
 *                      "Cfg.EcAddEntryComplete() for more information").\n
 *  \param[in] updateInRealtime 1 for update of entry data in realtime and
 *                      0 not to update data in realtime.\n
 * \note Example: Disable update of value in realtime for entry with name "AI_1" on
 * bus position 5.\n
 *  "Cfg.EcSetEntryUpdateInRealtime(5,AI_1,0)" //Command string to ecmcCmdParser.c\n
 */
int ecSetEntryUpdateInRealtime(
  uint16_t slavePosition,
  char    *entryIDString,
  int      updateInRealtime);

/** \brief Enable EtherCAT bus diagnostics.\n
  *
  * Diagnostics are made at three different levels:\n
  * 1. Slave level.\n
  * 2. Domain level.\n
  * 3. Master level.\n
  *
  * All three levels of diagnostics are enabled or disabled with this command.
  * If the diagnostics are enabled the motion axes are interlocked if any of
  * the above levels report issues. If the diagnostics are disabled the motion
  * axes are not interlocked even if there's alarms on the EtherCAT bus.\n
  *
  * \param[in] enable Enable diagnostics.\n
  * \return 0 if success or otherwise an error code.\n
  *
  * \note Example: Enable EtherCAT diagnostics.\n
  *  "Cfg.EcSetDiagnostics(1)" //Command string to ecmcCmdParser.c\n
  */
int ecSetDiagnostics(int enable);

/** \brief Set allowed bus cycles in row of none complete domain
 * data transfer.\n
 *
 * Allows a certain number of bus cycles of non-complete domain data
 * transfer before alarming.\n
 *
 *  \note Normally the application is correct configured all domain data
 *  transfers should be complete. This command should therefor only be
 *  used for testing purpose when the final configuration is not yet made.\n
  *
  * \param[in] cycles Number of cycles.\n
  *
  * \return 0 if success or otherwise an error code.\n
  *
  * \note Example: Set the allowd bus cycles of non complete domain data to
  * 10.\n
  *  "Cfg.EcSetDomainFailedCyclesLimit(10)" //Command string to ecmcCmdParser.c\n
  */
int ecSetDomainFailedCyclesLimit(int cycles);

/** \brief Reset error on all EtherCat objects.\n
 *
 * Resets error on the following object types:\n
 * 1. ecmcEc().\n
 * 2. ecmcEcSlave().\n
 * 3. ecmcEcSyncManager().\n
 * 4. ecmcEcPdo().\n
 * 5. ecmcEcSDO().\n
 * 6. ecmcEcEntry().\n
 *
  * \return 0 if success or otherwise an error code.\n
  *
  * \note Example: Reset EtherCAT errors.\n
  *  "Cfg.EcResetError()" //Command string to ecmcCmdParser.c\n
  */
int ecResetError();

/** \brief Enable diagnostic printouts from EtherCAT objects.\n
 *
 * Enables/Disables diagnostic printouts from:
 * 1. ecmcEc().\n
 * 2. ecmcEcSlave().\n
 * 3. ecmcEcSyncManager().\n
 * 4. ecmcEcPdo().\n
 * 5. ecmcEcSDO().\n
 * 6. ecmcEcEntry().\n
 *
 * \param[in] enable Enable diagnostic printouts.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Enable EtherCAT related diagnostic printouts.\n
 *  "Cfg.EcEnablePrintouts(1)" //Command string to ecmcCmdParser.c\n
 */
int ecEnablePrintouts(int value);

/** \brief Prints all hardware connected to selected master.\n
 *
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example:
 *  "EcPrintAllHardware()" //Command string to ecmcCmdParser.c\n
 */
int ecPrintAllHardware();

/** \brief Prints hardware configuration for a selected slave.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example:Print hardware configuration for slave 1
 *  "EcPrintSlaveConfig(1)" //Command string to ecmcCmdParser.c\n
 */
int ecPrintSlaveConfig(int slaveIndex);

/** \brief Links an EtherCAT entry to the ethecat master object for hardware
 *   status output\n
 *
 *  The output will be high when the EtherCAT master is without error code and
 *  otherwise zero.
 *
 *  \param[in] slaveIndex Position of the EtherCAT slave on the bus.\n
 *    slaveIndex = -1: Used to address the simulation slave. Only two
 *                     entries are configured, "ZERO" with default
 *                     value 0 and "ONE" with default value 1.\n
 *    slaveIndex = 0..65535: Addressing of normal EtherCAT slaves.\n
 *  \param[in] entryIdString String for addressing purpose (see command
 *                      "Cfg.EcAddEntryComplete() for more information").\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 *  \note Example 1: Link an EtherCAT entry configured as "OUTPUT_0" in slave 1 as
 *  status output for ethercat master.\n
 *   "Cfg.LinkEcEntryToEcStatusOutput(1,"OUTPUT_0")" //Command string to ecmcCmdParser.c\n
 */
int linkEcEntryToEcStatusOutput(int   slaveIndex,
                                char *entryIDString);

/** \brief Verfy slave at position
 *
 *  The command verifys that the actual slave at a certain position\
 *  have the correct alias, position, vendor id and product code.\n
 *
 *  \param[in] alias Alias of slave. Set to zero to disable.\n
 *  \param[in] slaveIn Position of the EtherCAT slave on the bus.\n
 *    slaveIndex = 0..65535: Addressing of normal EtherCAT slaves.\n
 *  \param[in] vendorId Identification value for slave vendor.\n
 *    vendorId = 0x2: Beckhoff.\n
 *    vendorId = 0x48554B: Kendrion Kuhnke Automation GmbH.\n
 *  \param productCode Product identification code.\n
 *    productCode=0x13ed3052: EL5101 incremental encoder input.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 *  \note Example: Verify that slave 3 is an EL5101\n
 *   "Cfg.EcVerifySlave(0,3,0x2,0x13ed3052)" //Command string to ecmcCmdParser.c\n
 */
int ecVerifySlave(uint16_t alias,  /**< Slave alias. */
                  uint16_t slavePos,   /**< Slave position. */
                  uint32_t vendorId,   /**< Expected vendor ID. */
                  uint32_t productCode  /**< Exp)*/);

# ifdef __cplusplus
}
# endif  // ifdef __cplusplus

#endif  /* ECMC_ETHERCAT_H_ */