#include "udp_receiver.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>

struct UmonUdpReceiver {
    SOCKET udp_socket;
    bool wsa_started;
};

UmonUdpReceiver *UmonUdpReceiverCreate(unsigned short port) {
    UmonUdpReceiver *receiver = calloc(1, sizeof(UmonUdpReceiver));
    if (receiver == NULL) {
        return NULL;
    }
    receiver->udp_socket = INVALID_SOCKET;

    WSADATA wsa_data;
    if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
        free(receiver);
        return NULL;
    }
    receiver->wsa_started = true;

    receiver->udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (receiver->udp_socket == INVALID_SOCKET) {
        UmonUdpReceiverDestroy(receiver);
        return NULL;
    }

    u_long non_blocking = 1;
    if (ioctlsocket(receiver->udp_socket, FIONBIO, &non_blocking) != 0) {
        UmonUdpReceiverDestroy(receiver);
        return NULL;
    }

    struct sockaddr_in address;
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(port);

    if (bind(receiver->udp_socket, (struct sockaddr *)&address, sizeof(address)) != 0) {
        UmonUdpReceiverDestroy(receiver);
        return NULL;
    }

    return receiver;
}

void UmonUdpReceiverDestroy(UmonUdpReceiver *receiver) {
    if (receiver == NULL) {
        return;
    }

    if (receiver->udp_socket != INVALID_SOCKET) {
        closesocket(receiver->udp_socket);
    }
    if (receiver->wsa_started) {
        WSACleanup();
    }

    free(receiver);
}

int UmonUdpReceiverPoll(UmonUdpReceiver *receiver, UmonUdpFrame *out_frame) {
    uint8_t buffer[sizeof(UmonUdpFrame)];
    int received = recvfrom(receiver->udp_socket, (char *)buffer, sizeof(buffer), 0, NULL, NULL);
    if (received == SOCKET_ERROR) {
        int socket_error = WSAGetLastError();
        if (socket_error == WSAEWOULDBLOCK) {
            return 0;
        }
        return -1;
    }

    if (received != (int)sizeof(UmonUdpFrame)) {
        return 0;
    }

    memcpy(out_frame, buffer, sizeof(UmonUdpFrame));
    return 1;
}
