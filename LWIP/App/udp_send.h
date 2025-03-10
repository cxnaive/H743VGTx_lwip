#ifndef _UDP_SEND_H
#define _UDP_SEND_H
#include "lwip/udp.h"

/** =========================================================
 * @brief  Initialize UDP PCB Struct
 * @param  remote_a, remote_b, remote_c, remote_d: remote ip address 
 * @param  local_port: local udp port
 * @param  remote_port: remote udp port
 * @retval UDP PCB Struct for do_udp_send
 * @remark struct udp_pcb* pcb = create_udp_send(192,168,1,255,5001,5002)
 * ========================================================*/
struct udp_pcb* create_udp_send(u32_t remote_a, u32_t remote_b, u32_t remote_c, u32_t remote_d, u16_t local_port, u16_t remote_port);

/** =========================================================
 * @brief  Send data via UDP protocol
 * @param  pcb: UDP PCB you got from create_udp_send
 * @param  data: data to send
 * @param  data_len: data length
 * @remark do_udp_send(pcb, "hello udp!", strlen("hello udp!"))
 * ========================================================*/
void do_udp_send(struct udp_pcb* pcb, void* data, u32_t data_len);

#endif