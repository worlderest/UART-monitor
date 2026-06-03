#ifndef UMON_UDP_RECEIVER_H
#define UMON_UDP_RECEIVER_H

#include "udp_protocol.h"

typedef struct UmonUdpReceiver UmonUdpReceiver;

UmonUdpReceiver *UmonUdpReceiverCreate(unsigned short port);
void UmonUdpReceiverDestroy(UmonUdpReceiver *receiver);
int UmonUdpReceiverPoll(UmonUdpReceiver *receiver, UmonUdpFrame *out_frame);

#endif
