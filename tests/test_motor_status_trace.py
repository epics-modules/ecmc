#!/usr/bin/env python3
"""Compile the production bounded trace with parameter-library stand-ins."""
from pathlib import Path
import subprocess,tempfile
s=(Path(__file__).resolve().parents[1] / 'devEcmcSup/motor/ecmcMotorRecordAxis.cpp').read_text()
a=s.index('void ecmcMotorRecordAxis::traceMotorStatus(');b=s.index('void ecmcMotorRecordAxis::report(',a)
code=r'''
#include <cstdio>
#include <cassert>
#include <cstring>
double ecmcMotorRecordgetNowTimeSecs(){return 1.;}
struct C { int motorStatusDone_=0,motorStatusProblem_=1,motorStatus_=2; int getIntegerParam(int,int,int *v){*v=2;return 0;} };
class ecmcMotorRecordAxis { public:
 C c,*pC_=&c; int axisNo_=6,waitNumPollsBeforeReady_=2; bool waitingForStatusAfterCommand_=false;
 unsigned observedMotionCommandExecuteCounter_=0;
 struct {bool moveReady=true,ecmcBusy=false; unsigned ecmcMotionCommandExecuteCounter=0; struct {int errorCode=0; struct {bool execute=false,enable=false,enabled=false;} statusWord_;} status_;} drvlocal;
 bool motorStatusTraceEnabled_=false; unsigned motorStatusTraceNext_=0,motorStatusTraceCount_=0; char motorStatusTrace_[128][512]={};
 void traceMotorStatus(const char*,int=-1); void dumpMotorStatusTrace(FILE*);
};
'''+s[a:b]+r'''
int main(){ecmcMotorRecordAxis a;a.traceMotorStatus("disabled");assert(a.motorStatusTraceCount_==0);a.motorStatusTraceEnabled_=true;for(int i=0;i<140;++i)a.traceMotorStatus("Done",i);assert(a.motorStatusTraceCount_==128);assert(strstr(a.motorStatusTrace_[a.motorStatusTraceNext_],"request=12 "));FILE *f=tmpfile();assert(f);a.dumpMotorStatusTrace(f);assert(ftell(f)>0);fclose(f);}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d);(p/'test.cpp').write_text(code)
 subprocess.run(['c++','-std=c++11','-Wall','-Wextra','-Werror',str(p/'test.cpp'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
print('Motor trace: production formatting, opt-in, ring wrap and dump passed')
