#include "lwip.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "usbd_cdc_if.h"
#include "udp_send.h"
#define MAX_SEND_LEN 1000

struct udp_pcb* create_udp_send(u32_t remote_a, u32_t remote_b, u32_t remote_c, u32_t remote_d, u16_t local_port, u16_t remote_port){
    err_t err;
    ip_addr_t remote_ipaddr;
    IP_ADDR4(&remote_ipaddr, remote_a, remote_b, remote_c, remote_d);
    struct udp_pcb* pcb = udp_new();
    err = udp_bind(pcb, IP_ADDR_ANY, local_port);
    if (err != ERR_OK){
        usb_printf("ip or port bind failed for port: %d\n", local_port);
        goto free_udp_pcb;
    }
    udp_connect(pcb, &remote_ipaddr, remote_port);
    return pcb;
free_udp_pcb:
    udp_remove(pcb);
}


void do_udp_send(struct udp_pcb* pcb, void* data, u32_t data_len){
    if (data_len > MAX_SEND_LEN) {
        usb_printf("send length too long!!!\n");
        return;
    }
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, data_len, PBUF_RAM);
    pbuf_take(p, data, data_len);
    udp_send(pcb, p);
    pbuf_free(p);
}