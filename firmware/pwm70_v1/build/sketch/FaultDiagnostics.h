#line 1 "D:\\OneDrive\\Desktop\\自走車\\firmware_builds\\pwm70_v1\\Orin_Interface_v1\\FaultDiagnostics.h"
#pragma once
#include <Arduino.h>
#include <stddef.h>
// Boot-lifetime evidence: never cleared by STOP, re-arm, or recovery attempts.
struct IoFailure {
  bool valid; uint32_t sequence, ms, generation; const char *operation;
  int channel, code; const char *codeKind; bool timeout; const char *duringStop;
  uint32_t causeBits, rawStatus, unknownInterruptCount; bool detailAvailable;
  uint32_t txStartUs, txEndUs, txDurationUs, txEndMs;
};
struct StopEvidence { bool valid; uint32_t ms, generation; const char *reason; };
struct TransactionTiming {uint32_t startUs,endUs,durationUs,endMs;};
extern TransactionTiming lastTransaction;
void transactionBegin();
void transactionEnd();
void confirmI2cRecovery(const char *method);
bool i2cRecoveryConfirmed();
extern IoFailure firstIoFailure, lastIoFailure;
extern StopEvidence firstFault, firstFaultStop;
extern uint32_t faultDiagnosticRevision;
void recordIoFailure(const char *operation,int channel,int code,const char *kind,bool timeout);
void diagnosticStopBegin(const char *reason,bool alreadyFault);
void diagnosticStopEnd(const char *reason,bool fault);
size_t formatFaultDiagnostics(char *buffer,size_t size);
void formatIoFailure(char *buffer,size_t size,const IoFailure &failure);
void telemetryIoFailure(const IoFailure &failure);
