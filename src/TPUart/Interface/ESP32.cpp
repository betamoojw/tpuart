#include "TPUart/Interface/ESP32.h"
#ifdef ARDUINO_ARCH_ESP32

namespace TPUart
{
namespace Interface
{

constexpr int TPUART_ESP32_RX_BUFFER_SIZE = 512;
constexpr int TPUART_ESP32_TX_BUFFER_SIZE = 512;
constexpr int TPUART_ESP32_EVENT_QUEUE_SIZE = 32;

ESP32::ESP32(int rx, int tx, uart_port_t uart) : _rx(rx), _tx(tx), _uart(uart) {}
ESP32::~ESP32() { end(); }

void ESP32::begin(uint32_t baud)
{
    if (_running) end();

    // Alle Felder ausdrücklich gesetzt - die beiden hinteren brauchen wir nicht, ohne sie warnt der
    // Compiler aber bei -Wextra (missing-field-initializers).
    uart_config_t uart_config = {
        .baud_rate = (int)baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {},
    };

    uart_param_config(_uart, &uart_config);

    // ACHTUNG, HÄNGT DEN KERN BEI FALSCHEN PINS: uart_set_pin() legt die Pin-Matrix um, ohne zu prüfen, ob
    // der Pin überhaupt frei ist. Liegt an ihm der im Gehäuse verbaute Flash oder PSRAM - beim
    // ESP32-PICO-V3-02 etwa GPIO 6-11 sowie 16 und 17, beim WROVER 16/17 -, dann findet der nächste
    // Cache-Miss keinen Flash mehr. Der Kern bleibt stehen, und weil auch der Panic-Handler erst aus dem
    // Flash gelesen werden müsste, kommt kein einziges Zeichen mehr heraus; nach 300ms setzt der
    // Interrupt-Watchdog still zurück (rst:0x8 TG1WDT_SYS_RESET). Wer hier einen Bootloop ohne jede
    // Ausgabe sieht, prüft ZUERST die Pins - im Code ist nichts zu finden. Siehe SESSION.md Abschnitt 66.
    uart_set_pin(_uart, _tx, _rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Rückgabewert prüfen: schlägt die Installation fehl, bliebe _eventQueue null, und ein späteres
    // xQueueReceive(nullptr, ...) in drainEventQueue() würde abstürzen. Relevant, weil die
    // Baudratenerkennung den Treiber laufend neu installiert, solange keine Busspannung anliegt.
    if (uart_driver_install(_uart, TPUART_ESP32_RX_BUFFER_SIZE, TPUART_ESP32_TX_BUFFER_SIZE, TPUART_ESP32_EVENT_QUEUE_SIZE, &_eventQueue, 0) != ESP_OK)
    {
        _eventQueue = nullptr;
        return; // _running bleibt false - alle Methoden liefern damit "nichts da"
    }

    // JEDES BYTE SOFORT HERAUSGEBEN - ohne diese zwei Zeilen ist die Pausenerkennung auf dem ESP32 blind.
    // Der Treiber lässt Empfangenes im 128-Byte-Hardware-FIFO liegen und schaufelt es erst in seinen
    // Ringpuffer, wenn entweder UART_FULL_THRESH_DEFAULT (120) Bytes beisammen sind oder die Leitung
    // UART_TOUT_THRESH_DEFAULT (10) Symbolzeiten still war - bei 19200 Baud sind das 5,7ms, also mehr als
    // unsere Frame-Ende-Schwelle von 2,6ms. Wir sehen dann 120 Bytes auf einen Schlag und danach eine
    // erfundene Pause; ein 263-Byte-Echo brach reproduzierbar bei genau 120 Bytes ab (SESSION.md 67).
    // Der Datalink Layer misst Pausen daran, wann ein Byte SICHTBAR wird - ein Interface darf deshalb nicht
    // stapeln. Beim RP2040 ergibt sich das von selbst, weil DMA jedes Byte sofort ablegt.
    // Kosten: ein Interrupt je Byte, bei 19200 Baud rund 1750 pro Sekunde. Belanglos.
    uart_set_rx_full_threshold(_uart, 1);
    uart_set_rx_timeout(_uart, 1);

    _byteTimeUs = baud > 0 ? (uint32_t)(11000000UL / baud) : 0;
    _lineBusyUntil = micros();
    _overflow = false;
    _running = true;
}

void ESP32::end()
{
    if (!_running) return;
    _running = false;

    uart_driver_delete(_uart);
    _eventQueue = nullptr;
}

// Die Event-Queue des Treibers wird nicht-blockierend im Poll-Pfad geleert - nur um Overflow-Ereignisse
// mitzubekommen. Die Daten selbst kommen ausschließlich über uart_read_bytes().
void ESP32::drainEventQueue()
{
    uart_event_t event;
    while (xQueueReceive(_eventQueue, (void *)&event, 0))
    {
        if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) _overflow = true;
    }
}

size_t ESP32::available()
{
    if (!_running) return 0;
    drainEventQueue();

    size_t len = 0;
    uart_get_buffered_data_len(_uart, &len);
    return len;
}

size_t ESP32::outstanding() const
{
    if (_byteTimeUs == 0) return 0;

    int32_t remaining = (int32_t)(_lineBusyUntil - micros());
    if (remaining <= 0) return 0;

    return (size_t)(((uint32_t)remaining + _byteTimeUs - 1) / _byteTimeUs);
}

// Der Platz bis zur erlaubten Tiefe, NICHT der freie Platz des Treiberpuffers - siehe Klassenkommentar.
// Der freie Platz wird trotzdem abgefragt und begrenzt zusätzlich nach unten: mehr als der Treiber annimmt,
// darf hier nie stehen, sonst würde uart_write_bytes() blockieren.
size_t ESP32::availableForWrite()
{
    if (!_running) return 0;

    size_t pending = outstanding();
    if (pending >= TPUART_TX_INTERFACE_BUFFER) return 0;

    size_t allowed = TPUART_TX_INTERFACE_BUFFER - pending;

    size_t len = 0;
    uart_get_tx_buffer_free_size(_uart, &len);

    return len < allowed ? len : allowed;
}

// Blockiert nur dann, wenn der Ringpuffer des Treibers voll ist - was hier doppelt ausgeschlossen ist: über
// die Tiefenbegrenzung und über den freien Platz, den availableForWrite() mit hereinrechnet. Der
// Treiber-Mutex ist unkritisch, solange nur ein Kontext schreibt.
// ACHTUNG: uart_write_bytes() ist nicht ISR-fähig. Soll der Datalink Layer auf dem ESP32 später aus einem
// Interrupt getaktet werden, muss das hier auf einen eigenen Ringpuffer + FreeRTOS-Task umgestellt werden.
bool ESP32::write(char value)
{
    if (!_running) return false;

    // Die Tiefenprüfung steht hier zusätzlich zu availableForWrite(): der Vertrag sagt zwar, dass der
    // Aufrufer vorher fragt, aber die Zusage "nie mehr als so viel unter uns" soll nicht daran hängen, dass
    // er das auch tut.
    if (outstanding() >= TPUART_TX_INTERFACE_BUFFER) return false;
    if (uart_write_bytes(_uart, &value, 1) != 1) return false;

    // Fahrplan fortschreiben: ist die Leitung noch belegt, hängt das Byte hinten dran, sonst beginnt es
    // jetzt.
    uint32_t now = micros();
    if ((int32_t)(_lineBusyUntil - now) < 0) _lineBusyUntil = now;
    _lineBusyUntil += _byteTimeUs;

    return true;
}

int ESP32::read()
{
    if (!available()) return -1;

    uint8_t value;
    return uart_read_bytes(_uart, &value, 1, 0) == 1 ? value : -1;
}

bool ESP32::overflow()
{
    if (!_running) return false;
    drainEventQueue();

    if (_overflow)
    {
        _overflow = false;
        return true;
    }
    return false;
}

void ESP32::flush()
{
    if (!_running) return;
    uart_flush(_uart);
}

} // namespace Interface
} // namespace TPUart

#endif
