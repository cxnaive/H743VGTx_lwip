#ifndef _CONFIG_H_
#define _CONFIG_H_
#include "stdint.h"
#define FLASH_VERSION (0)
struct __attribute__((packed)) flash_config_t {
    uint8_t version;
    uint32_t local_ip;
    uint16_t crc_16;
};

struct runtime_config_t {

};


struct config_t {
    struct runtime_config_t runtime_config;
    struct flash_config_t flash_config;
};
void init_config();
void save_flash_config(struct flash_config_t * flash_config);
void read_flash_config(struct flash_config_t * flash_config);

extern struct config_t config;
#endif