#pragma once

#include <Arduino.h>

typedef enum SerialControllerStatus
{
    SERIAL_CONTROLLER_CONNECTED,
    SERIAL_CONTROLLER_DISCONNECTED,
} SerialControllerStatus;

typedef void(SerialControllerStatusCb(SerialControllerStatus status));

void serial_controller_init();
void serial_controller_register_connection_callback(SerialControllerStatusCb *cb);
uint16_t serial_controller_get_val(uint8_t channel);
void serial_controller_send(uint8_t *buf, uint16_t length);
