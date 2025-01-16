#include "gptp.h"
#include "lwip/ip.h"
#include "netif/ethernet.h"
#include "lwip/stats.h"
#include "lwip/def.h"
#include "lwip/tcpip.h"
#include "lwip/snmp.h"
#include "usbd_cdc_if.h"
#include "ptpd.h"



void* callback_arg;
void setCallbackArg(void* arg){
    callback_arg = arg;
}
err_t gptp_tcpip_input(struct pbuf *p, struct netif *inp) {
    return tcpip_inpkt(p, inp, gptp_input);
}

err_t gptp_input(struct pbuf *p, struct netif *netif){
    struct eth_hdr *ethhdr;
    u16_t type;

    LWIP_ASSERT_CORE_LOCKED();

    if (p->len <= SIZEOF_ETH_HDR) {
    /* a packet with only an ethernet header (or less) is not valid for us */
        ETHARP_STATS_INC(etharp.proterr);
        ETHARP_STATS_INC(etharp.drop);
        MIB2_STATS_NETIF_INC(netif, ifinerrors);
        pbuf_free(p);
        return ERR_OK;
    }

    if (p->if_idx == NETIF_NO_INDEX) {
        p->if_idx = netif_get_index(netif);
    }
    ethhdr = (struct eth_hdr *)p->payload;
    u16_t msg_type = lwip_htons(ethhdr->type);
    // usb_printf("ethernet_input: dest:%02x:%02x:%02x:%02x:%02x:%02x, src:%02x:%02x:%02x:%02x:%02x:%02x, type:%04x\n",
    //            (unsigned char)ethhdr->dest.addr[0], (unsigned char)ethhdr->dest.addr[1], (unsigned char)ethhdr->dest.addr[2],
    //            (unsigned char)ethhdr->dest.addr[3], (unsigned char)ethhdr->dest.addr[4], (unsigned char)ethhdr->dest.addr[5],
    //            (unsigned char)ethhdr->src.addr[0],  (unsigned char)ethhdr->src.addr[1],  (unsigned char)ethhdr->src.addr[2],
    //            (unsigned char)ethhdr->src.addr[3],  (unsigned char)ethhdr->src.addr[4],  (unsigned char)ethhdr->src.addr[5],
    //            lwip_htons(ethhdr->type));
    if (msg_type == 0x88f7){
        // usb_printf("netRecvL2!\n");
        if(callback_arg != NULL){
            netRecvL2Callback(callback_arg, p, netif);
        }
        // pbuf_free(p);
        return ERR_OK;
    }
    

#if LWIP_ETHERNET
  if (netif->flags & (NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET)) {
    return ethernet_input(p, netif);
  } else
#endif /* LWIP_ETHERNET */
    return ip_input(p, netif);
}


/* Process an incoming message on the Layer2. */
void netRecvL2Callback(void* arg, struct pbuf *p, struct netif *netif){
	NetPath *netPath = (NetPath *) arg;
	

	octet_t* buf = (octet_t*) (p->payload + sizeof(struct eth_hdr));

	enum4bit_t messageType = (*(enum4bit_t*)(buf + 0)) & 0x0F;

    switch (messageType) {
        case 0xb:
        case 0xc:
            if (!netQPut(&netPath->generalQ, p)) {
                pbuf_free(p);
                ERROR("netRecvL2General: queue full\n");
                return;
            }
            // usb_printf("PTPv2 Packet General type:%02x!\n", messageType);
            break;
        case 0x0:
        case 0x2:
        case 0x3:
        case 0x8:
        case 0xa:
            if (!netQPut(&netPath->eventQ, p)) {
                pbuf_free(p);
                ERROR("netRecvL2Event: queue full\n");
                return;
            }
            // usb_printf("PTPv2 Packet Event type:%02x!\n", messageType);
            break;
    
        default:
            pbuf_free(p);
            ERROR("netRecvL2Event: queue full\n");
            return;
            break;
    }
	
    ptpd_alert();
}

ssize_t netRecv(octet_t *buf, TimeInternal *time, BufQueue *msgQueue)
{
	int i;
	int j;
	u16_t length;
	struct pbuf *p;
	struct pbuf *pcopy;

	/* Get the next buffer from the queue. */
	if ((p = (struct pbuf*) netQGet(msgQueue)) == NULL)
	{
		return 0;
	}

	/* Verify that we have enough space to store the contents. */
	if (p->tot_len > PACKET_SIZE)
	{
		ERROR("netRecv: received truncated message\n");
		pbuf_free(p);
		return 0;
	}

	/* Verify there is contents to copy. */
	if (p->tot_len == 0)
	{
		ERROR("netRecv: received empty packet\n");
		pbuf_free(p);
		return 0;
	}

	if (time != NULL)
	{
#if LWIP_PTP
		// usb_printf("recv ts: %u %u\n", p->time_sec, p->time_nsec);

		time->seconds = p->time_sec;
		time->nanoseconds = p->time_nsec;
#else
		getTime(time);
#endif
	}

	/* Get the length of the buffer to copy. */
	length = p->tot_len;

	/* Copy the pbuf payload into the buffer. */
	pcopy = p;
	j = 0;
	for (i = 0; i < length; i++)
	{
		// Copy the next byte in the payload.
		buf[i] = ((u8_t *)(pcopy->payload + sizeof(struct eth_hdr)))[j++];

		// Skip to the next buffer in the payload?
		if (j == pcopy->len)
		{
			// Move to the next buffer.
			pcopy = pcopy->next;
			j = 0;
		}
	}

	/* Free up the pbuf (chain). */
	pbuf_free(p);

	return length;
}


ssize_t netSendL2(NetPath *netPath, const octet_t *buf, int16_t  length, TimeInternal* time)
{
	// pbuf_free(p);
    err_t result;
	struct pbuf * p;

	/* Allocate the tx pbuf based on the current size. */
	p = pbuf_alloc(PBUF_LINK, length, PBUF_RAM);
	if (NULL == p) {
		ERROR("netSend: Failed to allocate Tx Buffer\n");
		goto fail01;
	}

    /* Copy the incoming data into the pbuf payload. */
	result = pbuf_take(p, buf, length);
	if (ERR_OK != result)
	{
		ERROR("netSend: Failed to copy data to Pbuf (%d)\n", result);
		goto fail02;
	}
#ifdef LWIP_PTP
	p->time_sec = PTP_TIMESTAMP_RECORD_MAGIC;
	p->time_nsec = PTP_TIMESTAMP_RECORD_MAGIC;
#endif

    // result = udp_sendto(pcb, p, (void *)addr, pcb->local_port);
    // result = 0;
    result = ethernet_output(netif_default, p, (const struct eth_addr*)(netif_default->hwaddr), (const struct eth_addr*)(netPath->peerMulticastMAC), ETHTYPE_PTP);
    // result = ethernet_output()
	if (ERR_OK != result)
	{
		ERROR("netSend: Failed to send data (%d)\n", result);
		goto fail02;
	}

    if (time != NULL)
	{
#if LWIP_PTP_TX
		time->seconds = p->time_sec;
		time->nanoseconds = p->time_nsec;
		usb_printf("send ts: %u %u\n", p->time_sec, p->time_nsec);
		// printf("netsend %u %u\n",p->time_sec,p->time_nsec);
#else
		/* TODO: use of loopback mode */
		/*
		time->seconds = 0;
		time->nanoseconds = 0;
		*/
		getTime(time);
#endif
		DBGV("netSend: %d sec %d nsec\n", time->seconds, time->nanoseconds);
	} else {
		DBGV("netSend\n");
	}

fail02:
	pbuf_free(p);

fail01:
	return length;
}


bool netShutdown(NetPath *netPath){
	setCallbackArg(NULL);
	return TRUE;
}

bool netInit(NetPath *netPath, PtpClock *ptpClock){
	netQInit(&netPath->eventQ);
	netQInit(&netPath->generalQ);
    //gptp MAC Addr
    netPath->peerMulticastMAC[0] = 0x01U;
    netPath->peerMulticastMAC[1] = 0x80U;
    netPath->peerMulticastMAC[2] = 0xc2U;
    netPath->peerMulticastMAC[3] = 0x00U;
    netPath->peerMulticastMAC[4] = 0x00U;
    netPath->peerMulticastMAC[5] = 0x0eU;
	setCallbackArg((void*)netPath);
	/* nothing to do with l2 layer GPTP*/
	return true;
}