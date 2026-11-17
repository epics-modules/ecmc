#!/usr/bin/env python3
"""Exercise path bounds, release-build thread startup and lookup NaN handling."""
# Expectation basis: INFERRED: bounds/cleanup; DOC: LUT interpolation; COMPAT/REVIEW: edge policies.
# Rationale, sources and open questions: EXPECTATIONS.md
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def method(source, signature):
    start = source.index(signature)
    return source[start:source.index('\n}', start) + 2] + '\n'


common = r'''
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <climits>
#include <cmath>
#include <limits>
#include <string>
#include <sstream>
#include <iomanip>
#include "ecmcErrorsList.h"
#define LOGERR(...) ((void)0)
#define LOGINFO4(...) ((void)0)
// Checks must execute even when the production code is built with NDEBUG.
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "Failed: %s at %d\n", #x, __LINE__); abort(); } } while (0)
'''
utils = (ROOT / 'devEcmcSup/com/ecmcAsynPortDriverUtils.cpp').read_text()
path_code = common + '#define EC_MAX_OBJECT_PATH_CHAR_LENGTH 256\n' + method(utils, 'int parseEcPath(') + r'''
int main() {
  int master = 7, slave = 8, bit = 9;
  char alias[256] = "unchanged";
  auto parse = [&](const std::string &path) {
    std::string copy = path;
    return parseEcPath(&copy[0], &master, &slave, alias, &bit);
  };
  CHECK(parse("ec0.s1.status.3") == 0);
  CHECK(master == 0 && slave == 1 && bit == 3 && strcmp(alias, "status") == 0);
  CHECK(parse("ec2.s-1.status") == 0);
  CHECK(master == 2 && slave == -1 && bit == -1);
  CHECK(parse("ec3.s15.sm2.p0.e0") == 0);
  CHECK(master == 3 && slave == 15 && bit == -1 && strcmp(alias, "sm2.p0.e0") == 0);
  CHECK(parse("ec3.s15.sm2.p0.e0.7") == 0);
  CHECK(master == 3 && slave == 15 && bit == 7 && strcmp(alias, "sm2.p0.e0") == 0);
  CHECK(parse("simEntry") == 0 && slave == -1 && master == 0 && bit == -1);
  CHECK(parse("simEntry.63") == 0 && bit == 63);
  CHECK(parse(std::string(255, 'a')) == 0 && strlen(alias) == 255);
  CHECK(parse("ec0.s1." + std::string(248, 'a')) == 0);
  const char *invalid[] = {"", "ec0.s1.status.3junk", "ec0.s1.",
    "ec0.s1.status.", "ec0.s1.status.-1", "ec0.s1.status.1.2",
    "ec2147483648.s1.status", "ec0.s-2147483649.status",
    "ec0.s1.status.2147483648", "ec0.s1.status.9999999999999999999999999",
    "ecX.s1.status", ".3", "simEntry.-1", "simEntry.3junk"};
  for (const char *path : invalid) {
    master = 7; slave = 8; bit = 9; strcpy(alias, "unchanged");
    CHECK(parse(path) == ERROR_MAIN_ECMC_COMMAND_FORMAT_ERROR);
    CHECK(master == 7 && slave == 8 && bit == 9 && strcmp(alias, "unchanged") == 0);
  }
  CHECK(parse(std::string(256, 'a')) != 0);
  CHECK(parse("ec0.s1." + std::string(300, 'a')) != 0);
  CHECK(parseEcPath(nullptr, &master, &slave, alias, &bit) != 0);
  char valid[] = "entry";
  CHECK(parseEcPath(valid, nullptr, &slave, alias, &bit) != 0);
  CHECK(parseEcPath(valid, &master, nullptr, alias, &bit) != 0);
  CHECK(parseEcPath(valid, &master, &slave, nullptr, &bit) != 0);
  CHECK(parseEcPath(valid, &master, &slave, alias, nullptr) != 0);
}
'''

thread = (ROOT / 'devEcmcSup/main/ecmcMainThread.cpp').read_text()
thread_prelude = r'''
#include <pthread.h>
#include <sched.h>
static int step, failAt, destroys, frees, allocations, creates;
static bool failAllocation;
static int next() { return ++step == failAt ? EINVAL : 0; }
static void *allocate(size_t n, size_t size) {
  if (failAllocation) return nullptr;
  ++allocations; return calloc(n, size);
}
static void release(void *p) { ++frees; free(p); }
static int attrInit(pthread_attr_t *) { return next(); }
static int attrInherit(pthread_attr_t *, int) { return next(); }
static int attrPolicy(pthread_attr_t *, int) { return next(); }
static int attrParam(pthread_attr_t *, const sched_param *) { return next(); }
static int attrStack(pthread_attr_t *, size_t) { return next(); }
static int attrDestroy(pthread_attr_t *) { ++destroys; return 0; }
static int create(pthread_t *, const pthread_attr_t *, void *(*)(void *), void *) {
  ++creates; return next();
}
static int setName(pthread_t, const char *) { return 0; }
#define calloc allocate
#define free release
#define pthread_attr_init attrInit
#define pthread_attr_setinheritsched attrInherit
#define pthread_attr_setschedpolicy attrPolicy
#define pthread_attr_setschedparam attrParam
#define pthread_attr_setstacksize attrStack
#define pthread_attr_destroy attrDestroy
#define pthread_create create
#define pthread_setname_np setName
#define ECMC_PRIO_HIGH 80
#define ECMC_PRIO_LOW 10
#define ECMC_RT_THREAD_NAME "test"
#define ECMC_STACK_SIZE 16384
#define ECMC_PRE_ALLOCATION_SIZE 4096
#define ECMC_ASYN_MAIN_PAR_STATUS_ID 0
static struct { int status; } threadDiag;
struct Param { void refreshParamRT(int) {} } param;
static Param *mainAsynParams[] = {&param};
static int epicsCalls, failures, locks, lockResult;
static void cyclic_task(void *) {}
static void *epicsThreadCreate(const char *, int priority, int, void (*)(void *), void *) {
  ++epicsCalls;
  CHECK(priority == (epicsCalls == 1 ? ECMC_PRIO_HIGH : ECMC_PRIO_LOW));
  return epicsCalls <= failures ? nullptr : &param;
}
static int lockMem(int) { ++locks; return lockResult; }
'''
a = thread.index('typedef void (*rtTHREADFUNC)')
b = thread.index('\nstruct timespec timespec_add', a)
thread_code = common + thread_prelude + thread[a:b] + method(thread, 'int startRTthread()') + r'''
int main() {
  for (int fail = 0; fail <= 6; ++fail) {
    step = destroys = frees = allocations = creates = 0; failAt = fail;
    rtThreadId id = rtThreadCreate("test", 80, 16384, cyclic_task, nullptr);
    CHECK(allocations == 1);
    if (!fail) {
      CHECK(id && step == 6 && destroys == 1 && creates == 1 && frees == 0);
      release(id);
    } else {
      CHECK(!id && step == fail && frees == 1);
      CHECK(destroys == (fail == 1 ? 0 : 1));
      CHECK(creates == (fail == 6 ? 1 : 0));
    }
  }
  failAllocation = true; step = 0;
  CHECK(!rtThreadCreate("test", 80, 16384, cyclic_task, nullptr) && step == 0);
  failAllocation = false; step = 0; failAt = 0;
  rtThreadId id = rtThreadCreate("test", 0, 16384, cyclic_task, nullptr);
  CHECK(id && step == 3); release(id);
  for (failures = 0; failures <= 2; ++failures) {
    epicsCalls = locks = 0; lockResult = 0;
    int rc = startRTthread();
    CHECK(epicsCalls == (failures ? 2 : 1));
    CHECK(threadDiag.status == (failures ? 0 : 1));
    CHECK(rc == (failures == 2 ? ERROR_MAIN_RT_THREAD_CREATE_FAILED : 0));
    CHECK(locks == (failures == 2 ? 0 : 1));
  }
  failures = epicsCalls = locks = 0; lockResult = ERROR_MAIN_RT_MEMORY_ALLOC_FAILED;
  CHECK(startRTthread() == ERROR_MAIN_RT_MEMORY_ALLOC_FAILED && locks == 1);
}
'''

lookup = (ROOT / 'devEcmcSup/misc/ecmcLookupTable.h').read_text()
lookup = lookup.replace('#include "ecmcError.h"', '''
class ecmcError {
 public: int setErrorID(const char *, const char *, int, int error) { return error; }
};
''')
lookup_code = common + lookup + r'''
int main(int argc, char **argv) {
  CHECK(argc == 2);
  ecmcLookupTable<double, double> table(argv[1]);
  // DOC: ecmcLookupTable.h class comment specifies interpolation and clamping.
  CHECK(table.getValidatedOK());
  CHECK(table.getValue(-20) == -2);
  CHECK(table.getValue(-10) == -2);
  CHECK(table.getValue(-5) == -1);
  CHECK(table.getValue(0) == 0);
  CHECK(table.getValue(5) == 2);
  CHECK(table.getValue(10) == 4);
  CHECK(table.getValue(20) == 4);
  // COMPAT/REVIEW F3: nonfinite-input results are a separate policy; safe
  // indexing alone does not specify propagation versus rejection/clamping.
  CHECK(table.getValue(-std::numeric_limits<double>::infinity()) == -2);
  CHECK(table.getValue(std::numeric_limits<double>::infinity()) == 4);
  CHECK(std::isnan(table.getValue(std::numeric_limits<double>::quiet_NaN())));
}
'''

with tempfile.TemporaryDirectory(prefix='ecmc-path-thread-lookup-') as directory:
    tmp = Path(directory)
    table = tmp / 'table.txt'
    table.write_text('-10 -2\n0 0\n10 4\n')
    for name, code in [('path', path_code), ('thread', thread_code), ('lookup', lookup_code)]:
        source = tmp / (name + '.cpp')
        source.write_text(code)
        # Thread checks exercise both assertion-enabled and release builds.
        for release in ([False, True] if name == 'thread' else [False]):
            executable = tmp / name
            subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-g', '-O1',
                            '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                            *(['-DNDEBUG'] if release else []),
                            '-I' + str(ROOT / 'devEcmcSup/main'), str(source),
                            '-o', str(executable)], check=True)
            subprocess.run([str(executable), str(table)], check=True)
print('Path parsing, thread startup and lookup regression checks passed.')
