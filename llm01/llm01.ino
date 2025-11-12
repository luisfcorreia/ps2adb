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

#define SERIAL_LOG 1
// ----------------------------- Configuração de pinos -----------------------------
const uint8_t PS2K_CLK = 3;   // Teclado PS/2 CLK
const uint8_t PS2K_DAT = 2;   // Teclado PS/2 DATA
const uint8_t PS2M_CLK = 7;   // Rato PS/2 CLK
const uint8_t PS2M_DAT = 6;   // Rato PS/2 DATA
const uint8_t ADB_PIN  = 12;   // Linha ADB (open-collector via transistor)
const uint8_t LED_PIN  = 13;

// ----------------------------- Parâmetros ADB -----------------------------
// Bit cell ≈ 100 µs. “1”: 35 µs LOW + 65 µs HIGH. “0”: 65 µs LOW + 35 µs HIGH.
// Tolerâncias: host ±3%, device ±30% (conforme AN591B). Ajustável conforme máquina.
/*
const uint16_t ADB_T_LOW_1_US = 35;
const uint16_t ADB_T_HIGH_1_US = 65;
const uint16_t ADB_T_LOW_0_US = 65;
const uint16_t ADB_T_HIGH_0_US = 35;
// Timings de enquadramento
const uint16_t ADB_T_START_US = 800;   // pulso de atenção/attn detetado do host (~0,8–1 ms típico)
const uint16_t ADB_T_SYNC_US  = 200;   // margem entre bits/stop (ajuste fino se necessário)

*/
// Conservador
const uint16_t ADB_T_LOW_1_US=40, ADB_T_HIGH_1_US=60;
const uint16_t ADB_T_LOW_0_US=66, ADB_T_HIGH_0_US=34;
const uint16_t ADB_T_START_US=900, ADB_T_SYNC_US=200;



// Endereços ADB dos dispositivos que vamos emular
const uint8_t ADB_ADDR_KBD = 2;  // teclado
const uint8_t ADB_ADDR_MSE = 3;  // rato

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

// ----------------------------- Utilitários GPIO ADB -----------------------------
inline void adb_line_low()   { pinMode(ADB_PIN, OUTPUT); digitalWrite(ADB_PIN, LOW); }
inline void adb_line_release(){ pinMode(ADB_PIN, INPUT_PULLUP); /* pull-up externo mantém HIGH */ }
inline uint8_t adb_line_read(){ return digitalRead(ADB_PIN); }

// Bitbang de um bit ADB (lado device)
void adb_write_bit(uint8_t bit1) {
  noInterrupts();
  if (bit1) { adb_line_low(); delayMicroseconds(35); adb_line_release(); delayMicroseconds(65); }
  else      { adb_line_low(); delayMicroseconds(65); adb_line_release(); delayMicroseconds(35); }
  interrupts();
}

// Envio de um byte ADB (MSB primeiro), inclui stop bit
void adb_write_byte(uint8_t b) {
  for (int i = 7; i >= 0; --i) {
    adb_write_bit((b >> i) & 1);
  }
  // Stop bit: “1”
  adb_write_bit(1);
}

// Envio de pacote de N bytes
void adb_write_packet(const uint8_t* data, uint8_t len) {
  noInterrupts();
  for (uint8_t i = 0; i < len; ++i) {
    for (int b = 7; b >= 0; --b) adb_write_bit((data[i] >> b) & 1);
    adb_write_bit(1); // stop
  }
  interrupts();
}

// Leitura de um byte do host (MSB primeiro). Muito simplificado: assume janela correta após ATTENTION.
uint8_t adb_read_byte_blocking() {
  uint8_t v = 0;
  for (int i = 7; i >= 0; --i) {
    // O host gera os níveis. Amostramos ao meio do bit cell.
    delayMicroseconds(50); // ~meio da célula de 100us
    uint8_t level = adb_line_read(); // 1=HIGH, 0=LOW
    // Interpretação simplista: HIGH predominante aproxima “1” (porque “1” tem 65us HIGH)
    // Aqui só usamos como relógio grosseiro. Em prática, deverias medir low vs high.
    // Para simplificar usamos transição média:
    v |= (level ? 1 : 0) << i;
    delayMicroseconds(50);
  }
  // Stop bit do host
  delayMicroseconds(100);
  return v;
}

// Aguarda pulso de atenção do host (queda longa da linha), retorna true se detetado
bool adb_wait_attention(uint32_t timeout_ms=50) {
  uint32_t t0 = millis();
  // Espera por LOW sustentado
  while (millis() - t0 < timeout_ms) {
    if (adb_line_read() == LOW) {
      // confirma que durou o suficiente
      delayMicroseconds(ADB_T_START_US);
      if (adb_line_read() == LOW) {
        // libertação
        while (adb_line_read() == LOW) { /* aguarda */ }
        delayMicroseconds(ADB_T_SYNC_US);
        return true;
      }
    }
  }
  return false;
}

// ----------------------------- PS/2 baixo nível (polling simples) -----------------------------
// PS/2: clock gerido pelo device. Amostramos na borda descendente.
// Implementação mínima, robusta para make/break e e0 prefixado.
bool ps2_read_byte(uint8_t clk, uint8_t dat, uint8_t &out) {
  // Espera start bit (DATA=LOW quando CLOCK desce)
  // Timeout simples
  uint32_t t0 = millis();
  while (digitalRead(clk) == HIGH) { if (millis() - t0 > 50) return false; }
  // 11 bits: start(0), 8 dados LSB->MSB, paridade, stop(1)
  uint8_t data = 0;
  // Sincroniza nas 8 quedas seguintes
  for (int i = 0; i < 11; ++i) {
    // Espera queda
    while (digitalRead(clk) == HIGH) { if (millis() - t0 > 50) return false; }
    delayMicroseconds(5); // pequena margem
    uint8_t bit = digitalRead(dat);
    if (i >= 1 && i <= 8) { // bits de dados
      data >>= 1;
      if (bit) data |= 0x80;
    }
    // Espera subida
    while (digitalRead(clk) == LOW) { if (millis() - t0 > 50) return false; }
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

  while (ps2_next_code(PS2K_CLK, PS2K_DAT, code)) {
    if (code == 0xF0) { break_next = true; continue; }
    if (code == 0xE0) { ext = true; continue; }

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
bool ps2_mouse_packet(int8_t &dx, int8_t &dy, uint8_t &btn) {
  uint8_t b0, b1, b2;
  if (!ps2_read_byte(PS2M_CLK, PS2M_DAT, b0)) return false;
  if (!ps2_read_byte(PS2M_CLK, PS2M_DAT, b1)) return false;
  if (!ps2_read_byte(PS2M_CLK, PS2M_DAT, b2)) return false;
  // PS/2: b0 bits: 0: left, 1: right, 2: middle, 4: sx, 5: sy
  int8_t dxs = (int8_t)((b0 & 0x10) ? (b1 | 0xFFFFFF00) : b1);
  int8_t dys = (int8_t)((b0 & 0x20) ? (b2 | 0xFFFFFF00) : b2);
  dx = dxs; dy = dys;
  btn = 0;
  if (b0 & 0x01) btn |= 0x04; // left -> bit2
  if (b0 & 0x04) btn |= 0x02; // middle -> bit1
  if (b0 & 0x02) btn |= 0x01; // right -> bit0
  return true;
}

void poll_ps2_mouse() {
  int8_t dx, dy; uint8_t b;
  // Poll muito simples: tenta ler “window” de 3 bytes se o clock baixar
  if (digitalRead(PS2M_CLK) == LOW) {
    if (ps2_mouse_packet(dx, dy, b)) {
      noInterrupts();
      mouse_dx += dx;
      mouse_dy += dy;
      mouse_buttons = b;
      interrupts();
#ifdef SERIAL_LOG
      Serial.print(F("[MSE] dx=")); Serial.print(dx); Serial.print(F(" dy=")); Serial.print(dy);
      Serial.print(F(" btn=")); Serial.println(b, BIN);
#endif
    }
  }
}

// ----------------------------- Resposta ADB aos “Talk/Listen” -----------------------------
// Comandos ADB: cmd = (addr<<4) | (op<<2) | reg, onde op: 0=Talk, 1=Listen, 2=Flush, 3=Reserved
// O host envia header e opcionalmente dados (Listen). Para simplificar, fazemos parsing leve.

uint8_t adb_read_command_byte() {
  return adb_read_byte_blocking();
}

void adb_reply_keyboard_reg0() {
  // ADB teclado reg0 devolve transições de teclas: 2 bytes por evento.
  // Implementação simplificada: se houver evento na FIFO, devolvemos 2 bytes com up/down + código; senão 0xFF 0xFF.
  uint8_t resp[2] = {0xFF, 0xFF};
  noInterrupts();
  if (kbd_head != kbd_tail) {
    uint8_t ev = kbd_fifo[kbd_tail]; kbd_tail = (kbd_tail + 1) & 7;
    bool up = (ev & 0x80) != 0;
    uint8_t code = ev & 0x7F;
    // Bits de reg0: [bit15 status primeiro; bits14..8 raw code; bit7 status segundo; bits6..0 segundo raw]
    // Para um único evento, colocamos no “primeiro”:
    resp[0] = (up ? 0x80 : 0x00) | (code >> 1);
    resp[1] = (uint8_t)((code & 0x01) << 7) | 0x7F; // “segundo” vazio
  }
  interrupts();
  adb_write_packet(resp, 2);
}

void adb_listen_keyboard_reg2(uint8_t data) {
  // LEDs: bits 0..2; guardamos e ignoramos visualmente
  kbd_leds = data & 0x07;
#ifdef SERIAL_LOG
  Serial.print(F("[ADB] KBD LEDs=")); Serial.println(kbd_leds, BIN);
#endif
}

void adb_reply_mouse_reg0() {
  // Rato handler 1/2: 2 bytes: [b xxx xxxx] [b yyy yyyy]; b=bit7=1 quando botão correspondente pressionado
  int16_t dx; int16_t dy; uint8_t b;
  noInterrupts();
  dx = mouse_dx; dy = mouse_dy; b = mouse_buttons;
  // Limita a -127..+127 e consome
  if (dx > 127) dx = 127; if (dx < -127) dx = -127;
  if (dy > 127) dy = 127; if (dy < -127) dy = -127;
  mouse_dx -= dx; mouse_dy -= dy;
  interrupts();

  uint8_t x = (uint8_t)(dx & 0x7F);
  uint8_t y = (uint8_t)(dy & 0x7F);
  uint8_t b1 = ((b & 0x04) ? 0x80 : 0x00) | x; // left no bit7 + X
  uint8_t b2 = ((b & 0x02) ? 0x80 : 0x00) | y; // middle no bit7 + Y
  // Nota: botão direito só aparece em formato extendido (3º byte). Mantemos protocolo simples a 2 bytes, botões left/mid.
  // Quem quiser mapear right→middle pode fazê-lo ajustando mouse_buttons.
  uint8_t pkt[2] = { b1, b2 };
  adb_write_packet(pkt, 2);
}

// Loop de atendimento ADB: espera ATTENTION, lê comando, responde se o endereço/reg nos diz respeito
void adb_service_loop_once() {
  if (!adb_wait_attention(2)) return; // nada novo

  uint8_t cmd = adb_read_command_byte();
  uint8_t addr = (cmd >> 4) & 0x0F;
  uint8_t op   = (cmd >> 2) & 0x03;
  uint8_t reg  = cmd & 0x03;

#ifdef SERIAL_LOG
  Serial.print(F("[ADB] cmd=")); Serial.print(cmd, HEX);
  Serial.print(F(" addr=")); Serial.print(addr);
  Serial.print(F(" op=")); Serial.print(op);
  Serial.print(F(" reg=")); Serial.println(reg);
#endif

  if (addr == ADB_ADDR_KBD) {
    if (op == 0 /*Talk*/) {
      if (reg == 0) adb_reply_keyboard_reg0();
      else {
        // Outras regs: devolve 0xFF
        uint8_t ff = 0xFF; adb_write_packet(&ff, 1);
      }
    } else if (op == 1 /*Listen*/) {
      // Lê um byte de dados
      uint8_t d = adb_read_byte_blocking();
      if (reg == 2) adb_listen_keyboard_reg2(d);
      // ACK vazio
    } else if (op == 2 /*Flush*/) {
      noInterrupts(); kbd_head = kbd_tail = 0; interrupts();
    }
  } else if (addr == ADB_ADDR_MSE) {
    if (op == 0 /*Talk*/) {
      if (reg == 0) adb_reply_mouse_reg0();
      else {
        uint8_t ff = 0xFF; adb_write_packet(&ff, 1);
      }
    } else if (op == 2 /*Flush*/) {
      noInterrupts(); mouse_dx = mouse_dy = 0; interrupts();
    }
  } else {
    // Não somos este device: ficar calado
  }
}

// ----------------------------- Setup e Loop -----------------------------
void setup() {
  pinMode(PS2K_CLK, INPUT_PULLUP);
  pinMode(PS2K_DAT, INPUT_PULLUP);
  pinMode(PS2M_CLK, INPUT_PULLUP);
  pinMode(PS2M_DAT, INPUT_PULLUP);
  pinMode(ADB_PIN, INPUT_PULLUP); // libertado; pull-up externo
  pinMode(LED_PIN, OUTPUT);
#ifdef SERIAL_LOG
  Serial.begin(115200);
  Serial.println(F("PS/2→ADB inicializado"));
#endif
}

void loop() {
  // 1) Servir PS/2
  poll_ps2_keyboard();
  poll_ps2_mouse();

  // 2) Servir ADB quando o host chamar
  adb_service_loop_once();

  // 3) Indicador LED simples
  static uint32_t lastBlink = 0;
  if (millis() - lastBlink > 250) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    lastBlink = millis();
  }
}
