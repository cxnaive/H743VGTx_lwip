#include "lwip.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/igmp.h"
#include "lwip/ip4_addr.h"
#include "usbd_cdc_if.h"
#include "udp_conn.h"
#define MAX_SEND_LEN 1000
#define MAX_RECV 20

int registered_recv_cnt = 0;
struct registered_recv_t
{
    struct udp_pcb *pcb;
    int buffer_max;
    recv_function recv_fn;
    uint8_t *buffer;
    void *args;
} recvs[MAX_RECV];

void raw_recv_udp(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    if (p != NULL) {
        int recv_regid = -1;
        for (int i = 0; i < registered_recv_cnt; ++i) {
            if (recvs[i].pcb == arg) {
                recv_regid = i;
                break;
            }
        }
        if (recv_regid == -1) {
            usb_printf("invalid recv package!!!!");
            goto free_udp_pcb;
        }
        uint8_t* recv_buf = recvs[recv_regid].buffer;
        if (p->tot_len > recvs[recv_regid].buffer_max - 1) {
            usb_printf("recv package too long!!!!");
            goto free_udp_pcb;
        }
        struct pbuf *q;
        u32_t recv_cnt = 0;
        for (q = p; q != NULL; q = q->next) {
            memcpy(recv_buf + recv_cnt, q->payload, q->len);
            recv_cnt += q->len;
        }
        recv_buf[recv_cnt] = 0;
        pbuf_free(p);
        // usb_printf("udp_recv: %s\n", (const char *)recv_buf);
        recvs[recv_regid].recv_fn(recvs[recv_regid].args, recvs[recv_regid].buffer, recv_cnt);
    }
free_udp_pcb:
    pbuf_free(p);
}

ip_addr_t create_ip_addr(u32_t a, u32_t b, u32_t c, u32_t d) {
    ip_addr_t now_ipaddr;
    IP_ADDR4(&now_ipaddr, a, b, c, d);
    return now_ipaddr;
}

struct udp_pcb *create_udp_send(u16_t local_port) {
    err_t err;

    struct udp_pcb *pcb = udp_new();
    err = udp_bind(pcb, IP_ADDR_ANY, local_port);
    if (err != ERR_OK) {
        usb_printf("ip or port bind failed for port: %d\n", local_port);
        goto free_udp_pcb;
    }
    // udp_connect(pcb, &remote_ipaddr, remote_port);
    return pcb;
free_udp_pcb:
    udp_remove(pcb);
}

struct udp_pcb *create_udp_recv(struct udp_pcb *send_pcb, ip_addr_t recv_addr, u16_t local_port, uint8_t* buffer, u32_t buffer_size, recv_function recv_fn, void *recv_fn_args) {
    err_t err;
    if (registered_recv_cnt >= MAX_RECV){
        usb_printf("too many recvs!!!");
        return send_pcb;
    }
    if (send_pcb == NULL) {
        send_pcb = udp_new();
        err = udp_bind(send_pcb, IP_ADDR_ANY, local_port);
        if (err != ERR_OK) {
            usb_printf("ip or port bind failed for port: %d\n", local_port);
            goto free_udp_pcb;
        }
    }
    if (ip4_addr_ismulticast(&recv_addr)) {
        err = igmp_joingroup(IP4_ADDR_ANY, &recv_addr);
        if (err != ERR_OK) {
            usb_printf("join_multicast failed: %d\n", recv_addr);
            goto free_udp_pcb;
        }
    }
    recvs[registered_recv_cnt].args = recv_fn_args;
    recvs[registered_recv_cnt].recv_fn = recv_fn;
    recvs[registered_recv_cnt].pcb = send_pcb;
    recvs[registered_recv_cnt].buffer = buffer;
    recvs[registered_recv_cnt].buffer_max = buffer_size;

    registered_recv_cnt++;
    udp_recv(send_pcb, raw_recv_udp, send_pcb);
    return send_pcb;
free_udp_pcb:
    udp_remove(send_pcb);
}

void do_udp_send(struct udp_pcb *pcb, ip_addr_t remote_addr, u16_t remote_port, void *data, u32_t data_len) {
    if (data_len > MAX_SEND_LEN) {
        usb_printf("send length too long!!!\n");
        return;
    }
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, data_len, PBUF_RAM);
    pbuf_take(p, data, data_len);
    udp_sendto(pcb, p, &remote_addr, remote_port);
    pbuf_free(p);
}