#ifndef UMON_SCORE_RECEIVER_H
#define UMON_SCORE_RECEIVER_H

#include "score_protocol.h"

typedef struct UmonScoreReceiver UmonScoreReceiver;

UmonScoreReceiver *UmonScoreReceiverCreate(unsigned short port);
void UmonScoreReceiverDestroy(UmonScoreReceiver *receiver);
int UmonScoreReceiverPoll(UmonScoreReceiver *receiver, UmonScoreEvent *out_event);

#endif
