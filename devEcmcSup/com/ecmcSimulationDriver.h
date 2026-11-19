#ifndef ECMC_SIMULATION_DRIVER_H_
#define ECMC_SIMULATION_DRIVER_H_

#include "ecmcIoDriver.h"
#include "ecmcDataAccess.h"
#include <map>
#include <memory>
#include <string>
#include <vector>

class ecmcAsynPortDriver;
class ecmcAsynDataItem;

// Memory-backed I/O only. Device behaviour belongs in external models.
class ecmcSimulationDriver final : public ecmcIoDriver {
public:
  struct Entry {
    ~Entry();
    uint64_t value = 0;
    int direction = 0;
    int syncManager = 0;
    int pdo = 0;
    int index = 0;
    int subIndex = 0;
    bool realtime = true;
    std::shared_ptr<ecmcDataAccess> access;
    ecmcAsynDataItem *simulationParameter = nullptr;
    std::string recordsPrefix;
    ecmcAsynDataItem *parameter = nullptr; // Registered/owned by asyn.
  };
  struct Device {
    int alias = 0;
    uint32_t vendor = 0, product = 0;
    uint32_t status = 0;
    std::map<std::string, std::unique_ptr<Entry>> entries;
    std::vector<Entry *> entryOrder;
    std::map<unsigned, std::vector<uint8_t>> objects;
    std::map<std::string, std::vector<int64_t>> settings;
  };

  ecmcSimulationDriver(int id, ecmcAsynPortDriver *port);
  ~ecmcSimulationDriver() override;
  int addDevice(int alias, int position, uint32_t vendor, uint32_t product);
  int addEntry(int position, uint32_t vendor, uint32_t product, int direction,
               int syncManager, int pdo, int index, int subIndex,
               ecmcEcDataType type, const char *name, bool realtime);
  int addDataItem(int position, const char *sourceName, size_t byteOffset,
                  size_t bitOffset, int direction, ecmcEcDataType type, const char *name);
  int importEntry(int position, const char *name, const std::shared_ptr<ecmcDataAccess> &source);
  int addAlias(int position, const char *name, const char *alias);
  Device *findDevice(int position);
  int positionByIndex(int index) const;
  Entry *findEntry(int position, const char *name);
  Entry *getEntry(int slaveIndex, int entryIndex);
  int findEntryIndex(int position, const char *name);
  int setObject(int position, int index, int subIndex, const void *data, size_t size);
  int setObjectHex(int position, int index, int subIndex, const char *text, size_t size);
  int setObjectTyped(int position, int index, int subIndex, ecmcEcDataType type, const char *value);
  int getObject(int position, int index, int subIndex, void *data, size_t size);
  int setSetting(int position, const std::string &name, std::vector<int64_t> values);
  int setValue(int position, const char *name, const char *value);
  int prepareSimulationParameter(int position, const char *name);
  const char *portName() const;
  const char *simulationPortName() const;
  int validate();
  int configurationError() const { return configurationError_; }
  void setHealthy(bool healthy) { *healthy_ = healthy; }
  int id() const { return id_; }

  bool isActive() const override { return active_; }
  void readInputs(timespec time, timespec offset) override;
  int updateIoStatus() override;
  void writeOutputs(timespec offset) override;
  void setTimeOffset(timespec offset) override { offset_ = offset; }
  bool timingReady() const override { return active_ && *healthy_; }
  void resetStartupErrors() override {} // Never hide an injected fault.
  void updateDiagnostics() override {}
  void slowExecute() override {}

private:
  int addEntryInternal(int position, uint32_t vendor, uint32_t product, int direction,
                       int syncManager, int pdo, int index, int subIndex,
                       ecmcEcDataType type, const char *name, bool realtime,
                       uint8_t *sharedData);
  static int parseValue(ecmcEcDataType type, const char *text, uint64_t *result);
  int registerDiagnostic(const std::string &name, uint32_t *value);
  void updateStatusWords();
  std::vector<int> deviceOrder_;
  int configurationError_ = 0;
  ecmcAsynDataItem *healthParameter_ = nullptr;
  int id_;
  ecmcAsynPortDriver *port_;
  ecmcAsynPortDriver *simulationPort_ = nullptr;
  std::shared_ptr<int> healthy_;
  bool active_ = false;
  timespec inputTime_{}, offset_{};
  std::map<int, Device> devices_;
  std::map<std::pair<int, std::string>, Entry *> aliases_;
  // Publication membership is frozen before RT; no cyclic name lookup.
  std::vector<Entry *> cyclicInputs_, cyclicOutputs_;
  std::vector<Device *> cyclicDevices_;
  std::vector<ecmcAsynDataItem *> diagnosticParameters_;
  uint32_t masterStatus_ = 0, slaveCount_ = 0, entryCount_ = 0, memmapCount_ = 0;
  uint32_t domainStatus_ = 0, failedCycles_ = 0;
  uint32_t statusOk_ = 0;
};

#endif
