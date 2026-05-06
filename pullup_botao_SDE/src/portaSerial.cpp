#include "portaSerial.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

portaSerial::portaSerial(int tx_pin, int rx_pin, int baud)
    : _tx((gpio_num_t)tx_pin),
      _rx((gpio_num_t)rx_pin),
      _period_us(1000000LL / baud),
      _tx_byte(0),
      _tx_idx(1),
      _tx_busy(false),
      _rx_byte(0),
      _rx_idx(-2),   // -2 = ocioso
      _rx_head(0),
      _rx_tail(0)
{
    // Pino TX: saída, nível alto (idle)
    gpio_reset_pin(_tx);
    gpio_set_direction(_tx, GPIO_MODE_OUTPUT);
    gpio_set_level(_tx, 1);

    // Pino RX: entrada com pull-up
    gpio_reset_pin(_rx);
    gpio_set_direction(_rx, GPIO_MODE_INPUT);
    gpio_set_pull_mode(_rx, GPIO_PULLUP_ONLY);

    // Timer de TX
    esp_timer_create_args_t args;
    memset(&args, 0, sizeof(args));
    args.callback = tx_cb;
    args.arg      = this;
    args.name     = "uart_tx";
    esp_timer_create(&args, &_tx_timer);

    // Timer de RX
    args.callback = rx_cb;
    args.name     = "uart_rx";
    esp_timer_create(&args, &_rx_timer);

    // Interrupção GPIO para detectar borda de descida (start bit)
    gpio_install_isr_service(0);   // retorna ESP_ERR_INVALID_STATE se já instalado (ok)
    gpio_set_intr_type(_rx, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add(_rx, rx_isr, this);
}

portaSerial::~portaSerial() {
    gpio_isr_handler_remove(_rx);
    esp_timer_stop(_tx_timer);
    esp_timer_delete(_tx_timer);
    esp_timer_stop(_rx_timer);
    esp_timer_delete(_rx_timer);
}

// ─── Transmissão ────────────────────────────────────────────────────────────

void portaSerial::envia(char c) {
    while (_tx_busy) {
        vTaskDelay(1);   // aguarda transmissão anterior terminar
    }
    _tx_byte = (uint8_t)c;
    _tx_idx  = 1;
    _tx_busy = true;

    // Start bit imediatamente
    gpio_set_level(_tx, 0);

    // Timer periódico cuidará dos bits de dados e do stop bit
    esp_timer_start_periodic(_tx_timer, _period_us);
}

void portaSerial::envia(const char* str) {
    while (*str) {
        envia(*str++);
    }
}

// Callback do timer de TX — chamado a cada _period_us
// Índices: 1-8 = D0..D7, 9 = stop bit, 10 = encerra
void portaSerial::tx_cb(void* arg) {
    portaSerial* self = static_cast<portaSerial*>(arg);
    int idx = self->_tx_idx;

    if (idx <= 8) {
        // Bit de dados (LSB primeiro)
        gpio_set_level(self->_tx, (self->_tx_byte >> (idx - 1)) & 1);
    } else if (idx == 9) {
        // Stop bit
        gpio_set_level(self->_tx, 1);
    } else {
        // idx == 10: quadro concluído
        esp_timer_stop(self->_tx_timer);
        self->_tx_busy = false;
        return;
    }
    self->_tx_idx = idx + 1;
}

// ─── Recepção ────────────────────────────────────────────────────────────────

// ISR: detecta borda de descida (start bit)
void IRAM_ATTR portaSerial::rx_isr(void* arg) {
    portaSerial* self = static_cast<portaSerial*>(arg);

    gpio_intr_disable(self->_rx);   // desabilita até fim do quadro
    self->_rx_idx = -1;             // aguardando confirmação do start bit

    // Agenda leitura no meio do start bit
    esp_timer_start_once(self->_rx_timer, self->_period_us / 2);
}

// Callback do timer de RX
// _rx_idx == -1 : meio do start bit (verificação)
// _rx_idx 0..7  : meio de cada bit de dado
void portaSerial::rx_cb(void* arg) {
    portaSerial* self = static_cast<portaSerial*>(arg);

    if (self->_rx_idx == -1) {
        // Verifica se o start bit é válido (linha ainda em nível baixo)
        if (gpio_get_level(self->_rx) == 0) {
            self->_rx_byte = 0;
            self->_rx_idx  = 0;
            esp_timer_start_once(self->_rx_timer, self->_period_us);
        } else {
            // Disparo falso — reabilita interrupção
            gpio_intr_enable(self->_rx);
        }
        return;
    }

    // Amostra bit de dado no meio do intervalo
    int bit = gpio_get_level(self->_rx);
    self->_rx_byte |= (uint8_t)(bit << self->_rx_idx);
    self->_rx_idx++;

    if (self->_rx_idx == 8) {
        // Byte completo
        self->rx_push(self->_rx_byte);
        self->_rx_idx = -2;   // volta ao estado ocioso
        gpio_intr_enable(self->_rx);
    } else {
        // Agenda próximo bit
        esp_timer_start_once(self->_rx_timer, self->_period_us);
    }
}

// ─── Buffer circular de recepção ─────────────────────────────────────────────

void portaSerial::rx_push(uint8_t b) {
    int next = (_rx_head + 1) % RX_BUF_SIZE;
    if (next != _rx_tail) {   // descarta se buffer cheio
        _rx_buf[_rx_head] = b;
        _rx_head = next;
    }
}

bool portaSerial::disponivel() {
    return _rx_head != _rx_tail;
}

char portaSerial::le() {
    while (!disponivel()) {
        vTaskDelay(1);   // bloqueia até receber dado
    }
    uint8_t b  = _rx_buf[_rx_tail];
    _rx_tail   = (_rx_tail + 1) % RX_BUF_SIZE;
    return (char)b;
}
