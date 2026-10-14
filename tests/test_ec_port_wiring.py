#!/usr/bin/env python3
"""Exercise the production startup check without EPICS or an EtherCAT bus.

Set ETHERCAT_INCLUDE to additionally compile against an installed ecrt.h.
"""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'devEcmcSup/ethercat/ecmcEc.cpp').read_text()
implementation = source[source.index('static void warnEcPortWiring('):
                        source.index('void ecmcEc::warnEcPortWiring()')]
main_source = (root / 'devEcmcSup/main/ecmcMainThread.cpp').read_text()
assert 'ec->warnEcPortWiring();' in main_source.split(
    'if (!ec->getScanBusyNotRT())', 1)[1].split('return 0;', 1)[0]
assert 'warnEcPortWiring(master_, masterIndex_);' not in source.split(
    'int ecmcEc::activate()', 1)[1].split('ecrt_master_activate(master_)', 1)[0]

stubs = r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <string>
#include <vector>
#ifdef TEST_REAL_ECRT
#include <ecrt.h>
#else
struct ec_master_t {};
constexpr unsigned EC_MAX_PORTS = 4;
constexpr int EC_MAX_STRING_LENGTH = 64;
enum { EC_PORT_NOT_IMPLEMENTED, EC_PORT_NOT_CONFIGURED, EC_PORT_EBUS, EC_PORT_MII };
struct ec_master_info_t { unsigned slave_count; bool scan_busy; };
struct ec_slave_info_t {
  struct Port {
    int desc;
    struct { bool link_up, loop_closed, signal_detected; } link;
    uint32_t receive_time;
  } ports[EC_MAX_PORTS];
  bool error_flag;
  char name[EC_MAX_STRING_LENGTH];
};
#endif
std::vector<ec_slave_info_t> slaves;
std::vector<std::string> buffered, console;
bool busy = false, masterFailure = false;
int failedPosition = -1;
struct ecmcEcSlave {
  bool enabled = true;
  bool getPortWiringCheckEnable() const { return enabled; }
};
struct ecmcEc {
  std::vector<ecmcEcSlave> configured;
  ecmcEcSlave *findSlave(unsigned int position) {
    if (position >= configured.size()) return nullptr;
    return &configured[position];
  }
};
ecmcEc ecmc;
int ecrt_master(ec_master_t *, ec_master_info_t *info) {
  info->slave_count = slaves.size(); info->scan_busy = busy;
  return masterFailure ? -1 : 0;
}
int ecrt_master_get_slave(ec_master_t *, uint16_t p, ec_slave_info_t *info) {
  if (p == failedPosition) return -1;
  *info = slaves.at(p); return 0;
}
void capture(std::vector<std::string>& out, const char *fmt, va_list args) {
  char text[1024]; vsnprintf(text, sizeof(text), fmt, args); out.emplace_back(text);
}
void ecmcLogBufferLogWarning(const char *fmt, ...) {
  va_list args; va_start(args, fmt); capture(buffered, fmt, args); va_end(args);
}
void ecmcRtLoggerLogWarning(const char *fmt, ...) {
  va_list args; va_start(args, fmt); capture(console, fmt, args); va_end(args);
}
'''
tests = r'''
int main() {
  int dummy;
  auto master = reinterpret_cast<ec_master_t *>(&dummy);
  auto check = [&](const char *expected) {
    ecmc.configured.assign(slaves.size(), ecmcEcSlave{});
    buffered.clear(); console.clear(); warnEcPortWiring(master, 2, &ecmc);
    assert(buffered.size() == (expected ? 1u : 0u));
    assert(console.size() == buffered.size());
    if (expected) {
      assert(buffered[0].find(expected) != std::string::npos);
      assert(console[0] == "WARNING: " + buffered[0] + "\n");
    }
  };
  check(nullptr); // Empty bus.
  ec_slave_info_t good = {};
  snprintf(good.name, sizeof(good.name), "EK1100");
  good.ports[0].desc = good.ports[2].desc = EC_PORT_MII;
  good.ports[1].desc = EC_PORT_EBUS;
  for (unsigned p = 0; p < 3; ++p) good.ports[p].link.link_up = true;
  good.ports[0].receive_time = 1000;
  good.ports[1].receive_time = 2000;
  good.ports[2].receive_time = 3000;
  slaves = {good}; check(nullptr); // Correctly wired coupler, both cables.
  auto &s = slaves[0];
  s.ports[2].link.link_up = false; check(nullptr); // End of line.
  s = good; s.ports[0].link.link_up = false;
  check("Port 0/IN has no link");
  s = good; s.ports[2].receive_time = 500;
  check("Suspected swapped IN/OUT cables");
  assert(buffered[0].find("ec2.s0 (EK1100)") != std::string::npos);
  assert(buffered[0].find("by 500 ns") != std::string::npos);
  // Rollover in either direction must not invert the diagnosis.
  s.ports[0].receive_time = 0xfffffff0u; s.ports[2].receive_time = 100;
  check(nullptr);
  s.ports[0].receive_time = 100; s.ports[2].receive_time = 0xfffffff0u;
  check("Suspected swapped");
  s.ports[2].receive_time = 0; check(nullptr);
  s.ports[0].receive_time = 0; check(nullptr); // No DC timestamps.
  s.ports[0].receive_time = s.ports[2].receive_time = 500; check(nullptr);
  s = good; s.ports[2].receive_time = 500;
  s.ports[2].link.loop_closed = true; check(nullptr);
  s.ports[2].link.loop_closed = false;
  s.ports[0].link.loop_closed = true; check(nullptr);
  s = good; s.ports[0].link.link_up = false;
  s.ports[0].desc = EC_PORT_EBUS; check(nullptr);
  s.ports[0].desc = EC_PORT_NOT_IMPLEMENTED; check(nullptr);
  s = good; s.ports[0].link.link_up = false;
  s.error_flag = 1; check(nullptr);
  s = good; busy = true; check("scan in progress"); busy = false;
  masterFailure = true; check("master information"); masterFailure = false;
  failedPosition = 0; check("slave information read failed"); failedPosition = -1;
  // A read failure must not prevent checking later slaves.
  slaves.push_back(good); slaves[1].ports[0].link.link_up = false;
  ecmc.configured.assign(slaves.size(), ecmcEcSlave{});
  failedPosition = 0; buffered.clear(); console.clear();
  warnEcPortWiring(master, 2, &ecmc); assert(buffered.size() == 2);
  assert(buffered[1].find("ec2.s1") != std::string::npos);
  ecmc.configured[1].enabled = false; buffered.clear(); console.clear();
  warnEcPortWiring(master, 2, &ecmc); assert(buffered.size() == 1);
  buffered.clear(); warnEcPortWiring(nullptr, 2, &ecmc); assert(buffered.empty());
}
'''
with tempfile.TemporaryDirectory() as directory:
    cpp = Path(directory) / 'test.cpp'
    binary = Path(directory) / 'test'
    cpp.write_text(stubs + implementation + tests)
    flags = []
    if os.environ.get('ETHERCAT_INCLUDE'):
        flags = ['-DTEST_REAL_ECRT', '-I' + os.environ['ETHERCAT_INCLUDE']]
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall',
                    '-Wextra', '-Werror', *flags, str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print('EtherCAT startup wiring diagnostics: PASS')
