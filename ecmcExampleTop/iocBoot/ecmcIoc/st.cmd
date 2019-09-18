#!../../bin/linux-x86_64/ecmcIoc

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/ecmcIoc.dbd"
ecmcIoc_registerRecordDeviceDriver pdbbase

## Load record instances
#dbLoadRecords("db/xxx.db","user=jhlee")

cd "${TOP}/iocBoot/${IOC}"
iocInit

## Start any sequence programs
#seq sncxxx,"user=jhlee"
