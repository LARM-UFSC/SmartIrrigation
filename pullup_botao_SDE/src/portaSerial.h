#pragma once

#include "driver/gpio.h"
#include "esp_timer.h"

class portaSerial {
public:
    portaSerial(int tx_pin, int rx_pin, int baud);
    ~portaSerial();

    void envia(char c);
    void envia(const char* str);
    char le();
    bool disponivel();

private:
    static const int RX_BUF_SIZE = 64;

    gpio_num_t _tx;
    gpio_num_t _rx;
    int64_t    _period_us;

    // TX
    esp_timer_handle_t _tx_timer;
    volatile uint8_t   _tx_byte;
    volatile int       _tx_idx;   // 1-8=dados, 9=stop, 10=fim
    volatile bool      _tx_busy;

    // RX
    esp_timer_handle_t _rx_timer;
    volatile uint8_t   _rx_byte;
    volatile int       _rx_idx;   // -1=start, 0-7=dados
    volatile uint8_t   _rx_buf[RX_BUF_SIZE];
    volatile int       _rx_head;
    volatile int       _rx_tail;

    static void           tx_cb(void* arg);
    static void           rx_cb(void* arg);
    static void IRAM_ATTR rx_isr(void* arg);

    void rx_push(uint8_t b);
};
