#line 1 "D:\\OneDrive\\Desktop\\自走車\\firmware_builds\\pwm70_v1\\Orin_Interface_v1\\FaultDiagnostics.cpp"
#include "FaultDiagnostics.h"
#include "Safety.h"
#include <Wire.h>
#include <stdio.h>
IoFailure firstIoFailure={},lastIoFailure={};
StopEvidence firstFault={},firstFaultStop={};
uint32_t faultDiagnosticRevision=0;
static const char *stopContext="";
static StopEvidence pendingStop={};
static uint32_t ioSequence=0;
TransactionTiming lastTransaction={};
static bool recovered=false;
static uint32_t recoveredMs=0,recoveredUs=0,recoveryDelayMs=0;
static const char *recoveryMethod="";
bool i2cRecoveryConfirmed(){return recovered;}
void transactionBegin(){lastTransaction.startUs=micros();}
void transactionEnd(){lastTransaction.endUs=micros();lastTransaction.durationUs=lastTransaction.endUs-lastTransaction.startUs;lastTransaction.endMs=millis();}
void confirmI2cRecovery(const char *method){
 if(!firstIoFailure.valid || recovered)return;
 recovered=true;recoveredMs=millis();recoveredUs=micros();
 recoveryDelayMs=recoveredMs-firstIoFailure.txEndMs;recoveryMethod=method;++faultDiagnosticRevision;
}

void recordIoFailure(const char *operation,int channel,int code,const char *kind,bool timeout) {
  IoFailure f={true,++ioSequence,millis(),safety.generation,operation,channel,code,kind,timeout,stopContext};
  #ifdef ORIN_WIRE_DETAIL_VERSION
  if(kind[0]=='W'){f.causeBits=orinWireCause;f.rawStatus=orinWireStatus;f.unknownInterruptCount=orinWireUnknownCount;f.detailAvailable=true;}
  #endif
  f.txStartUs=lastTransaction.startUs;f.txEndUs=lastTransaction.endUs;f.txDurationUs=lastTransaction.durationUs;f.txEndMs=lastTransaction.endMs;
  lastIoFailure=f;
  if(!firstIoFailure.valid)firstIoFailure=f;
  ++faultDiagnosticRevision;
  telemetryIoFailure(f);
}
void diagnosticStopBegin(const char *reason,bool alreadyFault) {
  stopContext=reason;
  if(!alreadyFault)pendingStop={true,millis(),safety.generation,reason};
}
void diagnosticStopEnd(const char *reason,bool fault) {
  if(fault && !firstFault.valid) {
    firstFault={true,millis(),safety.generation,reason};
    firstFaultStop=pendingStop;
    ++faultDiagnosticRevision;
  }
  stopContext="";
}
void formatIoFailure(char *b,size_t n,const IoFailure &f) {
  if(!f.valid){snprintf(b,n,"null");return;}
  snprintf(b,n,"{\"sequence\":%lu,\"ms\":%lu,\"generation\":%lu,\"operation\":\"%s\",\"address\":%u,\"channel\":%d,\"code\":%d,\"code_kind\":\"%s\",\"timeout\":%s,\"during_stop\":\"%s\",\"detail_available\":%s,\"buserr\":%s,\"arblos\":%s,\"unknown_irq\":%s,\"raw_status\":%lu,\"unknown_irq_count\":%lu,\"tx_start_us\":%lu,\"tx_end_us\":%lu,\"tx_duration_us\":%lu,\"tx_end_ms\":%lu}",
    (unsigned long)f.sequence,(unsigned long)f.ms,(unsigned long)f.generation,f.operation,(unsigned)PCA_ADDR,f.channel,f.code,f.codeKind,f.timeout?"true":"false",f.duringStop,f.detailAvailable?"true":"false",(f.causeBits&1)?"true":"false",(f.causeBits&2)?"true":"false",(f.causeBits&4)?"true":"false",(unsigned long)f.rawStatus,(unsigned long)f.unknownInterruptCount,(unsigned long)f.txStartUs,(unsigned long)f.txEndUs,(unsigned long)f.txDurationUs,(unsigned long)f.txEndMs);
}
static void stopJson(char *b,size_t n,const StopEvidence &f) {
  if(!f.valid){snprintf(b,n,"null");return;}
  snprintf(b,n,"{\"ms\":%lu,\"generation\":%lu,\"reason\":\"%s\"}",(unsigned long)f.ms,(unsigned long)f.generation,f.reason);
}
size_t formatFaultDiagnostics(char *buffer,size_t size) {
  char a[720],b[720],c[160],d[160],r[300];
  if(recovered)snprintf(r,sizeof(r),"{\"confirmed_ms\":%lu,\"confirmed_us\":%lu,\"observed_gap_ms\":%lu,\"method\":\"%s\",\"poll_interval_ms\":250,\"drive_resumed\":false}",(unsigned long)recoveredMs,(unsigned long)recoveredUs,(unsigned long)recoveryDelayMs,recoveryMethod);else snprintf(r,sizeof(r),"null");
  formatIoFailure(a,sizeof(a),firstIoFailure);formatIoFailure(b,sizeof(b),lastIoFailure);
  stopJson(c,sizeof(c),firstFault);stopJson(d,sizeof(d),firstFaultStop);
  return snprintf(buffer,size,"{\"type\":\"fault_diagnostics\",\"version\":2,\"ms\":%lu,\"revision\":%lu,\"first_fault\":%s,\"initiating_stop\":%s,\"first_i2c\":%s,\"last_i2c\":%s,\"first_recovery\":%s}",
    (unsigned long)millis(),(unsigned long)faultDiagnosticRevision,c,d,a,b,r);
}
