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

#define CHECK_STORAGE_RETURN_IF_ERROR(indexStorage)\
        {\
          if (indexStorage >= ECMC_MAX_DATA_STORAGE_OBJECTS ||\
              indexStorage < 0) {\
            LOGERR("ERROR: Data storage index out of range.\n");\
            return ERROR_MAIN_DATA_STORAGE_INDEX_OUT_OF_RANGE;\
          }\
          if (dataStorages[indexStorage] == NULL) {\
            LOGERR("ERROR: Data storage object NULL.\n");\
            return ERROR_MAIN_DATA_STORAGE_NULL;\
          }\
        }\

#define CHECK_LUT_RETURN_IF_ERROR(lutIndex)\
        {\
          if (lutIndex >= ECMC_MAX_LUTS ||\
              lutIndex < 0) {\
            LOGERR("ERROR: LUT list index out of range.\n");\
            return ERROR_LUT_INDEX_OUT_OF_RANGE;\
          }\
        }\


# ifdef __cplusplus
extern "C" {
# endif  // ifdef __cplusplus

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

/** \brief Load and create lookup table (LUT).\n
 *
 * \param[in] index Index of LUT.\n
 * \param[in] fileName Filename to load.\n
 *
 * \return 0 if success or otherwise an error code.\n
 *
 * \note Example: Load LUT to object index 1.\n
 *  "Cfg.LoadLUTFile(1,'test.lut')" //Command string to ecmcCmdParser.c\n
 */
int loadLUT(int index, char *fileName);

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
