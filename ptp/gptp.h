#ifndef _GPTP_H_
#define _GPTP_H_
#include "lwip/pbuf.h"
#include "lwip/netif.h"
#include "dep/ptpd_dep.h"

err_t gptp_tcpip_input(struct pbuf *p, struct netif *inp);
err_t gptp_input(struct pbuf *p, struct netif *netif);
void setCallbackArg(void* arg);
void netRecvL2Callback(void* arg, struct pbuf *p, struct netif *netif);
ssize_t netSendL2(NetPath *netPath, const octet_t *buf, int16_t  length, TimeInternal* time);
#endif