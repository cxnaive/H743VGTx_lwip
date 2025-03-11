#ifndef _UDP_SEND_H
#define _UDP_SEND_H
#include "lwip/udp.h"

typedef void (*recv_function)(void *arg, void* data, u32_t recv_len);

ip_addr_t create_ip_addr(u32_t a, u32_t b, u32_t c, u32_t d);
struct udp_pcb* create_udp_send(u16_t local_port);
struct udp_pcb *create_udp_recv(struct udp_pcb *send_pcb, ip_addr_t recv_addr, u16_t local_port, uint8_t* buffer, u32_t buffer_size, recv_function recv_fn, void *recv_fn_args);



void do_udp_send(struct udp_pcb* pcb, ip_addr_t remote_addr, u16_t remote_port, void* data, u32_t data_len);

#endif