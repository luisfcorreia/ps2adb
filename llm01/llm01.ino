/*
  PS/2 teclado+rato → ADB para Macintosh Classic
  Autor: Paulo-ready template por ChatGPT
  Licença: MIT (este ficheiro). Tabelas e ideias de temporização conforme créditos abaixo.

  Créditos e fontes:
  - Temporização ADB e formato de bit (bit cell 100 µs; “1” = 35 µs LOW + 65 µs HIGH, “0” = 65 µs LOW + 35 µs HIGH):
    Microchip App Note AN591B "Apple Desktop Bus (ADB)" (trechos temporais).
  - Resumo de temporização e introdução: “ADB: An Introduction” por Mateusz Łopaciuk.
  - Formato de pacotes de rato ADB (Handler 1/2 e 4), bits de botões e deltas de 7 bits: código-fonte do driver Linux adbmouse.c.
  - Endereço padrão do rato ($3), Handler ID $01, 2 bytes de movimento: RetroTechCollection wiki.
  - Mapeamento de keycodes ADB clássicos (baseados em tabelas comuns de keycodes do Mac): lista “Mac virtual keycodes” (A=0x00, S=0x01, ...).
  - PS/2 scan set 2: tabelas públicas (ex.: OSDev wiki e tabelas de scan codes set 2).

  Ver secção “Referências” no final para ligações.
*/

#include <Arduino.h>
#ifdef ARDUINO_ARCH_AVR
  #include <util/delay.h>
  #include <avr/pgmspace.h>
#else
  #define _delay_us(x) delayMicroseconds((unsigned int)(x))
  #ifndef PROGMEM
    #define PROGMEM
  #endif
#endif

#include "config.h"

// ----------------------------- ADB baixo nível (portado de adbduino) -----------------------------
// Vamos usar registos AVR diretamente para precisão de timings (como em adbduino), mas com o pino D12.
// Em Arduino Uno/Nano, D12 é PB4.
#ifdef ARDUINO_ARCH_AVR
  #define ADB_PORT        PORTB
  #define ADB_PINREG      PINB
  #define ADB_DDR         DDRB
  #define ADB_DATA_BIT    PB4
  // Controlos de linha (open-collector: saída a LOW para puxar; entrada para libertar)
  #define data_lo() { (ADB_DDR |=  (1<<ADB_DATA_BIT)); (ADB_PORT &= ~(1<<ADB_DATA_BIT)); }
  #define data_hi() (ADB_DDR &= ~(1<<ADB_DATA_BIT))
  #define data_in() (ADB_PINREG &   (1<<ADB_DATA_BIT))
#else
  // Fallback genérico (menos preciso) para não-AVR: usa pinMode/digitalWrite
  #define data_lo() { pinMode(ADB_PIN, OUTPUT); digitalWrite(ADB_PIN, LOW); }
  #define data_hi() { pinMode(ADB_PIN, INPUT_PULLUP); }
  #define data_in() (digitalRead(ADB_PIN))
#endif

static inline uint16_t wait_data_lo(uint16_t us)
{
  do {
    if ( !data_in() )
      break;
    _delay_us(1 - (6 * 1000000.0 / F_CPU));
  }
  while ( --us );
  return us;
}
static inline uint16_t wait_data_hi(uint16_t us)
{
  do {
    if ( data_in() )
      break;
    _delay_us(1 - (6 * 1000000.0 / F_CPU));
  }
  while ( --us );
  return us;
}

static inline void place_bit0(void)
{
  data_lo();
  _delay_us(65);
  data_hi();
  _delay_us(35);
}
static inline void place_bit1(void)
{
  data_lo();
  _delay_us(35);
  data_hi();
  _delay_us(65);
}
static inline void send_byte(uint8_t data)
{
  for (int i = 0; i < 8; i++) {
    if (data & (0x80 >> i))
      place_bit1();
    else
      place_bit0();
  }
}
static uint8_t inline adb_recv_cmd(uint8_t srq)
{
  uint8_t bits;
  uint16_t data = 0;

  // detetar attention & start bit do host
  if (!wait_data_lo(5000)) return 0;
  uint16_t lowtime = wait_data_hi(1000);
  if (!lowtime || lowtime > 500) {
    return 0;
  }
  wait_data_lo(100);

  for (bits = 0; bits < 8; bits++) {
    uint8_t lo = wait_data_hi(130);
    if (!lo) {
      goto out;
    }
    uint8_t hi = wait_data_lo(lo);
    if (!hi) {
      goto out;
    }
    hi = lo - hi;
    lo = 130 - lo;

    data <<= 1;
    if (lo < hi) {
      data |= 1;
    }
  }

  if (srq) {
    data_lo();
    _delay_us(250);
    data_hi();
  } else {
    // Stop bit normal low time é ~70us + pode ter SRQ de ~300us
    wait_data_hi(400);
  }

  return data;
out:
  return 0;
}

// Recebe um byte de dados ADB do host (após um comando Listen), usando o mesmo medidor de duty-cycle
static bool inline adb_recv_data_byte(uint8_t &out)
{
  uint8_t bits;
  uint16_t data = 0;

  for (bits = 0; bits < 8; bits++) {
    uint8_t lo = wait_data_hi(130);
    if (!lo) return false;
    uint8_t hi = wait_data_lo(lo);
    if (!hi) return false;
    hi = lo - hi;
    lo = 130 - lo;
    data <<= 1;
    if (lo < hi) data |= 1;
  }
  // Stop bit do host
  wait_data_hi(400);
  out = (uint8_t)data;
  return true;
}



// Endereços ADB dos dispositivos que vamos emular (parametrizáveis)

// ----------------------------- Buffers e estado -----------------------------
// Buffer simples de eventos do teclado: até 8 transições pendentes
volatile uint8_t kbd_fifo[8];
volatile uint8_t kbd_head = 0, kbd_tail = 0;

// Estado de modificadores e LEDs
volatile bool mod_shift = false, mod_ctrl = false, mod_alt = false, mod_cmd = false;
volatile uint8_t kbd_leds = 0; // bits 0..2 para LEDs (caps/num/scroll no ADB ext); aqui só indicativo

// Rato: deltas acumulados e botões
volatile int16_t mouse_dx = 0, mouse_dy = 0;
volatile uint8_t mouse_buttons = 0; // bit2=left, bit1=middle, bit0=right (convenção interna)

// Estado ADB (portado de adbduino)
uint8_t kbdsrq       = 0;
uint8_t kbdpending   = 0;
uint16_t kbdprev0    = 0;
uint16_t mousereg0   = 0;
uint16_t kbdreg0     = 0;
uint32_t kbskiptimer = 0;
uint8_t kbdskip      = 0;
uint8_t modifierkeys = 0xFF;
uint8_t mousesrq     = 0;
uint8_t mousepending = 0;

// ----------------------------- Utilitários (removidos os antigos de ADB bitbang — agora usamos os de cima) -----------------------------

// ----------------------------- PS/2 baixo nível (polling simples) -----------------------------
// PS/2: clock gerido pelo device. Amostramos na borda descendente.
// Implementação mínima, robusta para make/break e e0 prefixado.
bool ps2_read_byte(uint8_t clk, uint8_t dat, uint8_t &out) {
  // Versão não-bloqueante: se clock está HIGH, não há byte a iniciar
  if (digitalRead(clk) == HIGH) return false;
  // 11 bits: start(0), 8 dados LSB->MSB, paridade, stop(1)
  uint8_t data = 0;
  // Sincroniza nas 8 quedas seguintes
  for (int i = 0; i < 11; ++i) {
    // Espera queda
    uint32_t t0 = micros();
    while (digitalRead(clk) == HIGH) { if ((micros() - t0) > 2000) return false; }
    delayMicroseconds(5); // pequena margem
    uint8_t bit = digitalRead(dat);
    if (i >= 1 && i <= 8) { // bits de dados
      data >>= 1;
      if (bit) data |= 0x80;
    }
    // Espera subida
    t0 = micros();
    while (digitalRead(clk) == LOW) { if ((micros() - t0) > 2000) return false; }
  }
  out = data;
  return true;
}

// Leitura de um “scancode” completo (com prefixos), devolve código lógico:
// Retorna 0xF0 como “break”, 0xE0 como “extended”, ou byte normal.
bool ps2_next_code(uint8_t clk, uint8_t dat, uint8_t &code) {
  uint8_t b;
  if (!ps2_read_byte(clk, dat, b)) return false;
  code = b;
  return true;
}

// ----------------------------- Tabela de conversão PS/2(set2) → ADB -----------------------------
// Nota: ADB usa “raw key codes” do Mac clássico (A=0x00, S=0x01, D=0x02, ...).
// Tabela parcial: letras, dígitos da fileira superior, Enter, Esc, Backspace, Tab, Espaço, setas, modifiers.
struct MapEntry { uint8_t ps2; uint8_t adb; };
const MapEntry map_set2_to_adb[] PROGMEM = {
  // Letras
  {0x1C, 0x00}, // A
  {0x32, 0x0B}, // B
  {0x21, 0x08}, // C
  {0x23, 0x02}, // D
  {0x24, 0x0E}, // E
  {0x2B, 0x03}, // F
  {0x34, 0x05}, // G
  {0x33, 0x04}, // H
  {0x43, 0x22}, // I
  {0x3B, 0x26}, // J
  {0x42, 0x28}, // K
  {0x4B, 0x25}, // L
  {0x3A, 0x2E}, // M
  {0x31, 0x2D}, // N
  {0x44, 0x1F}, // O
  {0x4D, 0x23}, // P
  {0x15, 0x0C}, // Q
  {0x2D, 0x0F}, // R
  {0x1B, 0x01}, // S
  {0x2C, 0x11}, // T
  {0x3C, 0x20}, // U
  {0x2A, 0x09}, // V
  {0x1D, 0x0D}, // W
  {0x22, 0x07}, // X
  {0x35, 0x10}, // Y
  {0x1A, 0x06}, // Z

  // Dígitos fileira superior
  {0x16, 0x12}, // 1
  {0x1E, 0x13}, // 2
  {0x26, 0x14}, // 3
  {0x25, 0x15}, // 4
  {0x2E, 0x17}, // 5
  {0x36, 0x16}, // 6
  {0x3D, 0x1A}, // 7
  {0x3E, 0x1C}, // 8
  {0x46, 0x19}, // 9
  {0x45, 0x1D}, // 0

  // Pontuação principal
  {0x4E, 0x1B}, // '-' -> Mac '-'
  {0x55, 0x18}, // '=' -> Mac '='
  {0x54, 0x1E}, // '[' -> Mac ']'
  {0x5B, 0x2A}, // ']' -> Mac '\\' (aprox., depende de layout ISO/ANSI)
  {0x5D, 0x2A}, // '\' (set2 0x5D) → Mac '\\' (aprox.)
  {0x4C, 0x29}, // ';' -> Mac ';'
  {0x52, 0x27}, // '\'' -> Mac '\''
  {0x41, 0x2B}, // ',' -> Mac ','
  {0x49, 0x2F}, // '.' -> Mac '.'
  {0x4A, 0x2C}, // '/' -> Mac '/'

  // Controlo
  {0x76, 0x35}, // Esc
  {0x0D, 0x30}, // Tab
  {0x29, 0x31}, // Space
  {0x66, 0x33}, // Backspace (Delete à esquerda)
  {0x5A, 0x24}, // Enter (principal)
  {0x58, 0x39}, // Caps Lock

  // Modifiers (usar variantes “esquerda” em ADB simples)
  {0x12, 0x38}, // Left Shift
  {0x59, 0x38}, // Right Shift -> mesmo code em ADB simples
  {0x14, 0x3B}, // Left Ctrl -> Mac Control
  {0x11, 0x3A}, // Left Alt -> Mac Option
  // E0 14 -> Right Ctrl (mapeamos igual)
  // E0 11 -> Right Alt (Option)
  // E0 1F -> Left GUI (Windows) ≈ Mac Command (0x37)
  // E0 27 -> Right GUI ≈ Mac Command

  // Teclas de navegação (prefixo E0 no PS/2)
  // Setas, vamos tratá-las no caminho extended.
};

// Procura na tabela
int8_t ps2_to_adb(uint8_t ps2) {
  for (uint16_t i = 0; i < sizeof(map_set2_to_adb)/sizeof(map_set2_to_adb[0]); ++i) {
    if (pgm_read_byte(&map_set2_to_adb[i].ps2) == ps2) return pgm_read_byte(&map_set2_to_adb[i].adb);
  }
  return -1;
}

// Alguns extended mapeados explicitamente (prefixo E0)
int8_t ps2e0_to_adb(uint8_t ps2e0) {
  switch (ps2e0) {
    case 0x75: return 0x7E; // Up Arrow
    case 0x72: return 0x7D; // Down Arrow
    case 0x6B: return 0x7B; // Left Arrow
    case 0x74: return 0x7C; // Right Arrow
    case 0x14: return 0x3B; // Right Ctrl -> Control
    case 0x11: return 0x3A; // Right Alt  -> Option
    case 0x1F: return 0x37; // Left GUI   -> Command
    case 0x27: return 0x37; // Right GUI  -> Command
    case 0x70: return 0x72; // Insert -> Help (aprox.)
    case 0x6C: return 0x73; // Home  -> Home
    case 0x7D: return 0x74; // PgUp  -> PageUp
    case 0x71: return 0x75; // Delete-> ForwardDelete
    case 0x69: return 0x77; // End   -> End
    case 0x7A: return 0x79; // PgDn  -> PageDown
    default: return -1;
  }
}

// Enfileira transição ADB de tecla (raw code com bit7=up? No ADB, o host lê transições no reg0; aqui guardamos código e “up” num byte simples).
void kbd_enqueue(uint8_t adb_code, bool key_up) {
  uint8_t next = (kbd_head + 1) & 7;
  if (next == kbd_tail) return; // cheio
  // Convenção interna: bit7 = 1 para “key up”; restante = raw code
  kbd_fifo[kbd_head] = (key_up ? 0x80 : 0x00) | (adb_code & 0x7F);
  kbd_head = next;
}

// Processa bytes do teclado PS/2 e converte para eventos ADB
void poll_ps2_keyboard() {
  uint8_t code;
  static bool break_next = false;
  static bool ext = false;

  // Lê no máximo um código por chamada para evitar bloqueios
  if (ps2_next_code(PS2K_CLK, PS2K_DAT, code)) {
    if (code == 0xF0) { break_next = true; return; }
    if (code == 0xE0) { ext = true; return; }

    int8_t adb = -1;
    if (ext) adb = ps2e0_to_adb(code);
    else     adb = ps2_to_adb(code);

    bool key_up = break_next;
    break_next = false;
    ext = false;

    if (adb >= 0) {
      // Atualiza flags simples de modificadores
      if (adb == 0x38) mod_shift = !key_up;
      if (adb == 0x3B) mod_ctrl  = !key_up;
      if (adb == 0x3A) mod_alt   = !key_up;
      if (adb == 0x37) mod_cmd   = !key_up;

      kbd_enqueue((uint8_t)adb, key_up);
#ifdef SERIAL_LOG
      Serial.print(F("[KBD] ")); Serial.print(key_up ? F("UP ") : F("DN "));
      Serial.println(adb, HEX);
#endif
    } else {
#ifdef SERIAL_LOG
      Serial.print(F("[KBD] unmapped ps2 ")); Serial.print(ext ? F("E0 ") : F(""));
      Serial.println(code, HEX);
#endif
    }
  }
}

// ----------------------------- PS/2 Rato → ADB -----------------------------
// Leitura de pacotes PS/2 (3 bytes): [buttons | dx | dy], com sinais e overflow nos bits altos.
bool ps2_mouse_packet_raw(uint8_t &b0, uint8_t &b1, uint8_t &b2) {
  if (!ps2_read_byte(PS2M_CLK, PS2M_DAT, b0)) return false;
  if (!ps2_read_byte(PS2M_CLK, PS2M_DAT, b1)) return false;
  if (!ps2_read_byte(PS2M_CLK, PS2M_DAT, b2)) return false;
  return true;
}

void poll_ps2_mouse() {
  // Poll muito simples: tenta ler um pacote se o clock baixar
  if (digitalRead(PS2M_CLK) == LOW) {
    uint8_t b0, b1, b2;
    if (ps2_mouse_packet_raw(b0, b1, b2)) {
      // Construir mousereg0 com clamp configurável, preservando a semântica do adbduino
      // b0: bit0 L, bit1 R, bit2 M, bit4 X sign, bit5 Y sign
      // b1: X move (2's complement), b2: Y move (2's complement)
      int8_t dxs = (int8_t)((b0 & 0x10) ? (b1 | 0xFFFFFF00) : b1);
      int8_t dys = (int8_t)((b0 & 0x20) ? (b2 | 0xFFFFFF00) : b2);
      // Clamp a ±MOUSE_DELTA_CLAMP (6 bits)
      if (dxs > MOUSE_DELTA_CLAMP) dxs = MOUSE_DELTA_CLAMP;
      if (dxs < -MOUSE_DELTA_CLAMP) dxs = -MOUSE_DELTA_CLAMP;
      if (dys > MOUSE_DELTA_CLAMP) dys = MOUSE_DELTA_CLAMP;
      if (dys < -MOUSE_DELTA_CLAMP) dys = -MOUSE_DELTA_CLAMP;

      uint8_t magX6 = (uint8_t)((dxs < 0 ? -dxs : dxs) & 0x3F);
      uint8_t magY6 = (uint8_t)((dys < 0 ? -dys : dys) & 0x3F);

      mousereg0 = 0x80; // base fixa
      // Botão esquerdo com inversão configurável
      bool leftPressed = PS2_B0_LEFT_ACTIVE_HIGH ? ((b0 & 0x01) != 0) : ((b0 & 0x01) == 0);
      if (!leftPressed) mousereg0 |= 0x8000; // como no adbduino
      // Y: sinal conforme padrão adbduino (flip versus PS/2), magnitude clamp
      if (magY6) {
        mousereg0 |= (~(b0) & 0x20) << 9; // preserva semântica de sinal do adbduino
        mousereg0 |= (uint16_t)(magY6) << 8;
      }
      // X: sinal do PS/2 diretamente, magnitude clamp
      if (magX6) {
        mousereg0 |= (b0 & 0x10) << 2;
        mousereg0 |= magX6;
      }
      mousepending = 1;
      // Guardar estado simples para debug e possivel talk reg2 (opcional)
      noInterrupts();
      mouse_dx += dxs;
      mouse_dy += dys;
      mouse_buttons = ((b0 & 0x01)?0x04:0) | ((b0 & 0x04)?0x02:0) | ((b0 & 0x02)?0x01:0);
      interrupts();
#ifdef SERIAL_LOG
      Serial.print(F("[MSE] raw x=")); Serial.print((int)b1);
      Serial.print(F(" y=")); Serial.print((int)b2);
      Serial.print(F(" b0=")); Serial.println(b0, BIN);
#endif
    }
  }
}

// ----------------------------- Envio PS/2 (host→device) -----------------------------
// Implementação leve com pinMode/digitalWrite; suficiente para LEDs e comandos básicos.

static bool ps2_host_write_byte(uint8_t clkPin, uint8_t datPin, uint8_t val) {
  // 1) Inibir clock por >=100us para pedir envio
  pinMode(clkPin, OUTPUT); digitalWrite(clkPin, LOW);
  delayMicroseconds(150);
  // 2) Levar data a LOW enquanto mantemos clock LOW
  pinMode(datPin, OUTPUT); digitalWrite(datPin, LOW);
  // 3) Libertar clock (device irá gerar clock)
  pinMode(clkPin, INPUT_PULLUP);

  // 4) Enviar 8 bits LSB-first em bordas de clock do device
  uint8_t parity = 0;
  for (uint8_t i = 0; i < 8; i++) {
    // Espera clock LOW
    uint32_t t0 = micros();
    while (digitalRead(clkPin) == HIGH) { if ((micros()-t0) > 2000) return false; }
    // Define bit em data
    if (val & (1 << i)) { digitalWrite(datPin, HIGH); parity++; }
    else                { digitalWrite(datPin, LOW); }
    // Espera clock HIGH (fim do bit)
    t0 = micros();
    while (digitalRead(clkPin) == LOW) { if ((micros()-t0) > 2000) return false; }
  }

  // 5) Paridade (odd)
  {
    uint32_t t0 = micros();
    while (digitalRead(clkPin) == HIGH) { if ((micros()-t0) > 2000) return false; }
    if ((parity & 1) == 0) digitalWrite(datPin, HIGH); else digitalWrite(datPin, LOW);
    t0 = micros();
    while (digitalRead(clkPin) == LOW) { if ((micros()-t0) > 2000) return false; }
  }

  // 6) Stop bit: libertar DATA
  pinMode(datPin, INPUT_PULLUP);
  // Device deverá gerar um ACK (linha data LOW) na próxima janela
  // Espera opcional por ACK
  uint32_t t0 = micros();
  while (digitalRead(clkPin) == HIGH) { if ((micros()-t0) > 2000) break; }
  // ACK é lido quando data=LOW durante um ciclo; não validamos estritamente aqui.

  // 7) Fim
  return true;
}

static void ps2_keyboard_send(uint8_t b) {
  ps2_host_write_byte(PS2K_CLK, PS2K_DAT, b);
}

static void ps2_keyboard_set_leds_from_adb(uint8_t adb_leds) {
  // ADB talk reg2 no adbduino envia invertido, LEDs ON quando bit=0.
  // Converter para PS/2 0xED: bits: 0=Scroll, 1=Num, 2=Caps (1=ON)
  uint8_t ps2 = 0;
  // Heurística: se bit for 0 em ADB, LED ON
  if ((adb_leds & 0x04) == 0) ps2 |= 0x01; // Scroll
  if ((adb_leds & 0x01) == 0) ps2 |= 0x02; // Num
  if ((adb_leds & 0x02) == 0) ps2 |= 0x04; // Caps
  // Sequência: 0xED, depois máscara LEDs
  for (uint8_t attempt = 0; attempt < 2; ++attempt) {
    ps2_keyboard_send(0xED);
    bool ok = true;
#if PS2_REQUIRE_ACK
    // Esperar ACK (0xFA)
    uint8_t ack;
    uint32_t t0 = millis();
    while (!ps2_read_byte(PS2K_CLK, PS2K_DAT, ack)) { if (millis() - t0 > 10) { ok = false; break; } }
    if (ok && ack != 0xFA) ok = false;
#endif
    if (!ok) continue; // retry
    ps2_keyboard_send(ps2);
#if PS2_REQUIRE_ACK
    ok = true; t0 = millis();
    while (!ps2_read_byte(PS2K_CLK, PS2K_DAT, ack)) { if (millis() - t0 > 10) { ok = false; break; } }
    if (ok && ack != 0xFA) ok = false;
    if (!ok) continue; // retry de toda a sequência
#endif
    break; // sucesso ou ignore ACK
  }
}

// Setup de rato PS/2 (reset, sample rate, resolution, scaling, enable)
static void ps2_mouse_setup_if_enabled() {
#if PS2M_SETUP_ENABLE
  auto sendM = [](uint8_t b){ ps2_host_write_byte(PS2M_CLK, PS2M_DAT, b); };
  auto waitAck = []() -> bool {
#if PS2_REQUIRE_ACK
    uint8_t ack;
    uint32_t t0 = millis();
    while (!ps2_read_byte(PS2M_CLK, PS2M_DAT, ack)) { if (millis() - t0 > 20) return false; }
    return ack == 0xFA;
#else
    return true;
#endif
  };

  // Reset
  #if SERIAL_LOG
    Serial.println(F("[PS2M] Reset"));
  #endif
  sendM(0xFF); (void)waitAck();
  // Após reset, o rato pode enviar 0xAA e ID: ignoramos por simplicidade
  // Sample rate
  #if SERIAL_LOG
    Serial.print(F("[PS2M] SampleRate=")); Serial.println(PS2M_SAMPLE_RATE, HEX);
  #endif
  sendM(0xF3); (void)waitAck();
  sendM(PS2M_SAMPLE_RATE); (void)waitAck();
  // Resolution
  #if SERIAL_LOG
    Serial.print(F("[PS2M] Resolution=")); Serial.println(PS2M_RESOLUTION, HEX);
  #endif
  sendM(0xE8); (void)waitAck();
  sendM(PS2M_RESOLUTION); (void)waitAck();
  // Scaling 1:1
  #if SERIAL_LOG
    Serial.println(F("[PS2M] Scaling 1:1"));
  #endif
  sendM(0xE6); (void)waitAck();
  // Enable
  #if SERIAL_LOG
    Serial.println(F("[PS2M] Enable"));
  #endif
  sendM(0xF4); (void)waitAck();
#endif
}

// ----------------------------- Resposta ADB aos “Talk/Listen” -----------------------------
// Comandos ADB: cmd = (addr<<4) | (op<<2) | reg, onde op: 0=Talk, 1=Listen, 2=Flush, 3=Reserved
// O host envia header e opcionalmente dados (Listen). Para simplificar, fazemos parsing leve.

// Nota: Removemos o serviço ADB antigo e vamos usar a recepção/envio fiáveis como em adbduino no loop principal
// Helpers de inibição PS/2 (coordenar janelas ADB)
static inline void ps2_inhibit_all() {
  // Teclado
  pinMode(PS2K_CLK, OUTPUT); digitalWrite(PS2K_CLK, LOW);
  // Rato
  pinMode(PS2M_CLK, OUTPUT); digitalWrite(PS2M_CLK, LOW);
}
static inline void ps2_release_all() {
  pinMode(PS2K_CLK, INPUT_PULLUP);
  pinMode(PS2M_CLK, INPUT_PULLUP);
}

// ----------------------------- Setup e Loop -----------------------------
void setup() {
  pinMode(PS2K_CLK, INPUT_PULLUP);
  pinMode(PS2K_DAT, INPUT_PULLUP);
  pinMode(PS2M_CLK, INPUT_PULLUP);
  pinMode(PS2M_DAT, INPUT_PULLUP);
  // Linha ADB como entrada (libertada)
  ADB_DDR &= ~(1 << ADB_DATA_BIT);
  pinMode(LED_PIN, OUTPUT);
#ifdef SERIAL_LOG
  Serial.begin(115200);
  Serial.println(F("PS/2→ADB inicializado"));
#endif
  // Setup do rato PS/2 (opcional)
  ps2_inhibit_all();
  ps2_mouse_setup_if_enabled();
  ps2_release_all();
}

void loop() {
  // 1) Servir PS/2
  poll_ps2_keyboard();
  poll_ps2_mouse();

  // Preparar eventos de teclado para ADB, seguindo estilo do adbduino
  if (!kbdpending) {
    noInterrupts();
    bool has = (kbd_head != kbd_tail);
    uint8_t ev = 0;
    if (has) { ev = kbd_fifo[kbd_tail]; kbd_tail = (kbd_tail + 1) & 7; }
    interrupts();
    if (has) {
      uint8_t maccode = ev & 0x7F;
      bool key_up = (ev & 0x80) != 0;
      if (key_up) maccode |= 0x80;
      if (maccode == 0xFF) {
        kbdskip = 1;
      } else {
        kbdprev0 = maccode;
      }
      kbdreg0 = ((uint16_t)maccode << 8) | 0xFF;
      kbdpending = 1;
    }
  }

  // 2) ADB: se temos dados pendentes, escutar comando do host e responder
  uint8_t cmd = 0;
  if (mousepending || kbdpending) {
    cmd = adb_recv_cmd(mousesrq | kbdsrq);
  }

  // Rato no endereço 3
  if (((cmd >> 4) & 0x0F) == ADB_ADDR_MSE) {
    switch (cmd & 0x0F) {
      case 0xC: // talk register 0
        if (mousepending) {
          ps2_inhibit_all();
          _delay_us(180);
          ADB_DDR |= 1 << ADB_DATA_BIT; // saída
          place_bit1(); // start bit
          uint8_t mouse_b0 = (mousereg0 >> 8) & 0xFF;
          uint8_t mouse_b1 = mousereg0 & 0xFF;
          send_byte(mouse_b0);
          send_byte(mouse_b1);
#if ADB_MOUSE_HANDLER == 4
          // Stub Handler 4: adicionar 3º byte com botão direito no bit7 (resto 0)
          uint8_t mouse_b2 = (mouse_buttons & 0x01) ? 0x80 : 0x00; // right -> bit7
          send_byte(mouse_b2);
#endif
          place_bit0(); // stop bit
          ADB_DDR &= ~(1 << ADB_DATA_BIT); // liberta
          mousepending = 0;
          mousesrq = 0;
          ps2_release_all();
        }
        break;
      default:
        // ignorar
        break;
    }
  } else {
    if (mousepending) mousesrq = 1; // pedir serviço
  }

  // Teclado no endereço 2
  if (((cmd >> 4) & 0x0F) == ADB_ADDR_KBD) {
    switch (cmd & 0x0F) {
      case 0xC: // talk register 0
        if (kbdpending) {
          if (kbdskip) {
            kbdskip = 0;
            // envia keyup do anterior para evitar stuck
            kbdprev0 |= 0x80;
            kbdreg0 = (kbdprev0 << 8) | 0xFF;
            kbskiptimer = millis();
          } else if (millis() - kbskiptimer < 90) {
            kbdpending = 0;
            break;
          }

          kbdsrq = 0;
          ps2_inhibit_all();
          _delay_us(180);
          ADB_DDR |= 1 << ADB_DATA_BIT; // saída
          place_bit1();
          send_byte((kbdreg0 >> 8) & 0xFF);
          send_byte(kbdreg0 & 0xFF);
          place_bit0();
          ADB_DDR &= ~(1 << ADB_DATA_BIT);
          kbdpending = 0;
          ps2_release_all();
        }
        break;
      case 0xA: { // listen register 2 (host → LEDs)
        uint8_t d;
        if (adb_recv_data_byte(d)) {
          ps2_inhibit_all();
          // refletir no teclado PS/2
          ps2_keyboard_set_leds_from_adb(d);
#ifdef SERIAL_LOG
          Serial.print(F("[ADB] KBD Listen Reg2 d=")); Serial.println(d, HEX);
#endif
          ps2_release_all();
        }
        break;
      }
      case 0xE: { // talk register 2 (modifiers/leds)
        // Construir adbleds de forma simples (tudo apagado=1 em ADB clássico invertido)
        uint8_t adbleds = 0xFF;
        // Sem estado de LEDs real, mantemos 0xFF
        ps2_inhibit_all();
        _delay_us(180);
        ADB_DDR |= 1 << ADB_DATA_BIT;
        place_bit1();
        // modifierkeys: vamos sintetizar a partir dos flags
        uint8_t mods = 0xFF;
        // Bits limpados quando ativos (como em adbduino)
        if (mod_cmd)    mods &= ~(1);
        if (mod_alt)    mods &= ~(2);
        if (mod_shift)  mods &= ~(4);
        if (mod_ctrl)   mods &= ~(8);
        send_byte(mods);
        send_byte(adbleds);
        place_bit0();
        ADB_DDR &= ~(1 << ADB_DATA_BIT);
        ps2_release_all();
        break;
      }
      default:
        break;
    }
  } else {
    if (kbdpending) kbdsrq = 1;
  }

  // 3) Indicador LED simples
  static uint32_t lastBlink = 0;
  if (millis() - lastBlink > 250) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    lastBlink = millis();
  }
}
