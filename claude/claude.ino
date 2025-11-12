/*
 * PS/2 Keyboard and Mouse to Macintosh ADB Interface
 * For Arduino ATmega328P (Uno/Nano)
 * 
 * Wiring:
 * PS/2 Keyboard: CLK=Pin2 (INT0), DATA=Pin3
 * PS/2 Mouse:    CLK=Pin4, DATA=Pin5
 * ADB:           DATA=Pin7 with 1kΩ pullup to 5V
 * 
 * Based on working mouse code from adbduino
 */

#include <avr/interrupt.h>

// Pin definitions
#define PS2_KB_CLK    3
#define PS2_KB_DATA   2
#define PS2_MS_CLK    7
#define PS2_MS_DATA   6
#define ADB_PIN       11

// ADB timing constants (microseconds)
#define ADB_BIT_CELL  100
#define ADB_SYNC_TIME 70
#define ADB_BIT1_LOW  35
#define ADB_BIT0_LOW  65

// PS/2 Keyboard state
volatile uint8_t kb_bit_count = 0;
volatile uint16_t kb_data_word = 0;
volatile bool kb_frame_ready = false;
uint8_t kb_extended = 0;
uint8_t kb_release = 0;

// PS/2 Mouse state  
uint8_t mouse_buffer[4];
uint8_t mouse_idx = 0;
int8_t mouse_x = 0;
int8_t mouse_y = 0;
uint8_t mouse_btn = 0;
bool mouse_ready = false;

// ADB state
uint8_t adb_kbd_data[2] = {0xFF, 0xFF}; // Key up by default
uint8_t adb_mouse_data[2] = {0x80, 0x80}; // Button up, centered
bool adb_service_request = false;

// PS/2 to ADB scancode map (standard keys)
const uint8_t PROGMEM ps2_adb_table[] = {
  0xFF,0x35,0xFF,0xFF,0xFF,0xFF,0xFF,0x7A,0xFF,0x7B,0xFF,0xFF,0xFF,0x30,0x32,0xFF,
  0xFF,0xFF,0x38,0xFF,0x3A,0x0C,0x35,0xFF,0xFF,0xFF,0x06,0x07,0x00,0x08,0x01,0xFF,
  0xFF,0x09,0x0B,0x2D,0x2E,0x21,0x1D,0xFF,0xFF,0x31,0x0A,0x22,0x2F,0x04,0x1E,0xFF,
  0xFF,0x0D,0x05,0x23,0x20,0x02,0x1F,0xFF,0xFF,0xFF,0x24,0x10,0x03,0x11,0x0E,0xFF,
  0xFF,0x25,0x12,0x0F,0x13,0x14,0xFF,0xFF,0xFF,0x26,0x27,0x17,0x16,0x15,0xFF,0xFF,
  0xFF,0xFF,0x28,0xFF,0x18,0x19,0xFF,0xFF,0x39,0x3B,0x24,0x29,0x2A,0x2B,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x33,0xFF,0xFF,0x59,0xFF,0x5B,0x5C,0xFF,0xFF,0xFF,
  0x52,0x53,0xFF,0x54,0xFF,0x55,0x56,0x57,0x58,0x7C,0x4B,0xFF,0x56,0x43,0xFF,0xFF
};

void setup() {
    
  Serial.begin(57600);
  Serial.println("PS/2 to ADB Converter");
  
  init_ps2_keyboard();
  delay(100);
  init_ps2_mouse();
  delay(100);
  init_adb();
  
  Serial.println(F("Initialization complete"));
}

void loop() {
  process_keyboard();
  process_mouse();
  
  // Poll ADB commands from Mac
  check_adb_attention();
}

//=== PS/2 KEYBOARD ===

void init_ps2_keyboard() {
  pinMode(PS2_KB_CLK, INPUT_PULLUP);
  pinMode(PS2_KB_DATA, INPUT_PULLUP);
  
  Serial.print(F("Waiting for keyboard..."));
  delay(1000); // Give keyboard time to power up
  Serial.println(F("OK"));
  
  // Try to reset keyboard (non-blocking)
  Serial.print(F("Reset keyboard..."));
  if (ps2_write_safe(PS2_KB_CLK, PS2_KB_DATA, 0xFF)) {
    Serial.println(F("OK"));
    delay(100);
  } else {
    Serial.println(F("SKIP"));
  }
  
  // Enable scanning
  Serial.print(F("Enable scanning..."));
  if (ps2_write_safe(PS2_KB_CLK, PS2_KB_DATA, 0xF4)) {
    Serial.println(F("OK"));
  } else {
    Serial.println(F("SKIP"));
  }
  delay(50);
  
  // Enable interrupt - use ISR directly, not attachInterrupt
  EIMSK |= (1 << INT0);     // Enable INT0
  EICRA |= (1 << ISC01);    // Trigger on falling edge
  EICRA &= ~(1 << ISC00);
  
  Serial.println(F("Keyboard initialized"));
}

// Keyboard clock interrupt handler
ISR(INT0_vect) {
  static uint8_t parity = 0;
  uint8_t val = (PIND >> PS2_KB_DATA) & 1; // Direct port read for speed
  
  if (kb_bit_count == 0) {
    // Start bit
    if (val == 0) {
      kb_data_word = 0;
      parity = 0;
      kb_bit_count++;
    }
  } else if (kb_bit_count >= 1 && kb_bit_count <= 8) {
    // Data bits
    kb_data_word >>= 1;
    if (val) {
      kb_data_word |= 0x80;
      parity ^= 1;
    }
    kb_bit_count++;
  } else if (kb_bit_count == 9) {
    // Parity bit
    if (val) parity ^= 1;
    kb_bit_count++;
  } else if (kb_bit_count == 10) {
    // Stop bit
    if (val && parity) {
      kb_data_word >>= 1;
      kb_frame_ready = true;
    }
    kb_bit_count = 0;
  }
}

void process_keyboard() {
  if (!kb_frame_ready) return;
  
  uint8_t scancode = kb_data_word & 0xFF;
  kb_frame_ready = false;
  
  // Handle special codes
  if (scancode == 0xE0) {
    kb_extended = 1;
    return;
  }
  if (scancode == 0xE1) {
    kb_extended = 2; // Pause/Break
    return;
  }
  if (scancode == 0xF0) {
    kb_release = 1;
    return;
  }
  
  // Translate to ADB
  uint8_t adb_code = translate_ps2_to_adb(scancode, kb_extended);
  
  if (adb_code != 0xFF) {
    if (kb_release) {
      adb_kbd_data[0] = adb_code | 0x80; // Key up
      Serial.print(F("Key UP: "));
    } else {
      adb_kbd_data[0] = adb_code; // Key down
      Serial.print(F("Key DN: "));
    }
    adb_kbd_data[1] = 0xFF;
    adb_service_request = true;
    
    Serial.print(F("PS2=0x"));
    Serial.print(scancode, HEX);
    Serial.print(F(" ADB=0x"));
    Serial.println(adb_code, HEX);
  }
  
  kb_extended = 0;
  kb_release = 0;
}

uint8_t translate_ps2_to_adb(uint8_t ps2, uint8_t extended) {
  if (extended == 1) {
    // Extended scancodes
    switch (ps2) {
      case 0x1F: return 0x37; // Left Command/GUI
      case 0x27: return 0x37; // Right Command/GUI  
      case 0x14: return 0x36; // Right Control
      case 0x11: return 0x3A; // Right Alt/Option
      case 0x70: return 0x72; // Insert
      case 0x6C: return 0x73; // Home
      case 0x7D: return 0x74; // Page Up
      case 0x71: return 0x75; // Delete
      case 0x69: return 0x77; // End
      case 0x7A: return 0x79; // Page Down
      case 0x75: return 0x3E; // Up Arrow
      case 0x6B: return 0x3B; // Left Arrow
      case 0x72: return 0x3D; // Down Arrow
      case 0x74: return 0x3C; // Right Arrow
      case 0x4A: return 0x4B; // Keypad /
      case 0x5A: return 0x4C; // Keypad Enter
      default: return 0xFF;
    }
  }
  
  // Standard scancodes
  if (ps2 < 128) {
    return pgm_read_byte(&ps2_adb_table[ps2]);
  }
  return 0xFF;
}

//=== PS/2 MOUSE ===

void init_ps2_mouse() {
  pinMode(PS2_MS_CLK, INPUT_PULLUP);
  pinMode(PS2_MS_DATA, INPUT_PULLUP);
  
  Serial.print(F("Waiting for mouse..."));
  delay(1000); // Give mouse time to power up
  Serial.println(F("OK"));
  
  // Reset mouse
  Serial.print(F("Reset mouse..."));
  if (ps2_write_safe(PS2_MS_CLK, PS2_MS_DATA, 0xFF)) {
    Serial.println(F("OK"));
    delay(100);
    ps2_read_byte(PS2_MS_CLK, PS2_MS_DATA); // ACK
    ps2_read_byte(PS2_MS_CLK, PS2_MS_DATA); // 0xAA
    ps2_read_byte(PS2_MS_CLK, PS2_MS_DATA); // 0x00
  } else {
    Serial.println(F("SKIP"));
  }
  
  // Enable data reporting
  Serial.print(F("Enable mouse..."));
  if (ps2_write_safe(PS2_MS_CLK, PS2_MS_DATA, 0xF4)) {
    Serial.println(F("OK"));
  } else {
    Serial.println(F("SKIP"));
  }
  delay(50);
  
  mouse_idx = 0;
  Serial.println(F("Mouse initialized"));
}

void process_mouse() {
  // Check if data available
  unsigned long start = millis();
  while (digitalRead(PS2_MS_CLK) == HIGH && (millis() - start < 20)) {
    // Wait for clock to go low
  }
  
  if (digitalRead(PS2_MS_CLK) == LOW) {
    uint8_t byte = ps2_read_byte(PS2_MS_CLK, PS2_MS_DATA);
    
    if (byte == 0) return; // Invalid
    
    if (mouse_idx == 0 && (byte & 0x08) == 0) {
      // Invalid start byte
      return;
    }
    
    mouse_buffer[mouse_idx++] = byte;
    
    if (mouse_idx >= 3) {
      mouse_idx = 0;
      
      // Parse packet
      mouse_btn = mouse_buffer[0] & 0x03;
      mouse_x = (int8_t)mouse_buffer[1];
      mouse_y = -(int8_t)mouse_buffer[2]; // Invert Y for Mac
      
      // Apply sign extension
      if (mouse_buffer[0] & 0x10) mouse_x |= 0xFFFFFF00;
      if (mouse_buffer[0] & 0x20) mouse_y |= 0xFFFFFF00;
      
      // Clamp to 7-bit range
      if (mouse_x > 63) mouse_x = 63;
      if (mouse_x < -64) mouse_x = -64;
      if (mouse_y > 63) mouse_y = 63;
      if (mouse_y < -64) mouse_y = -64;
      
      // Build ADB format: [button:1 | reserved:1 | X:6] [reserved:1 | Y:7]
      adb_mouse_data[0] = ((mouse_btn & 1) ? 0x00 : 0x80) | (mouse_x & 0x7F);
      adb_mouse_data[1] = mouse_y & 0x7F;
      
      mouse_ready = true;
      adb_service_request = true;
      
      Serial.print(F("Mouse: B="));
      Serial.print(mouse_btn);
      Serial.print(F(" X="));
      Serial.print(mouse_x);
      Serial.print(F(" Y="));
      Serial.println(mouse_y);
    }
  }
}

//=== PS/2 LOW LEVEL ===

// Safe version of ps2_write with timeout
bool ps2_write_safe(uint8_t clk_pin, uint8_t data_pin, uint8_t data) {
  uint8_t parity = 1;
  unsigned long timeout;
  
  noInterrupts();
  
  // Request to send
  pinMode(clk_pin, OUTPUT);
  digitalWrite(clk_pin, LOW);
  delayMicroseconds(100);
  
  pinMode(data_pin, OUTPUT);
  digitalWrite(data_pin, LOW);
  
  pinMode(clk_pin, INPUT_PULLUP);
  
  // Wait for device to take control (with timeout)
  timeout = millis();
  while (digitalRead(clk_pin)) {
    if (millis() - timeout > 50) {
      interrupts();
      pinMode(data_pin, INPUT_PULLUP);
      return false;
    }
  }
  
  // Send data bits
  for (uint8_t i = 0; i < 8; i++) {
    if (data & 1) {
      pinMode(data_pin, INPUT_PULLUP);
      parity ^= 1;
    } else {
      pinMode(data_pin, OUTPUT);
      digitalWrite(data_pin, LOW);
    }
    
    timeout = millis();
    while (!digitalRead(clk_pin)) {
      if (millis() - timeout > 50) {
        interrupts();
        return false;
      }
    }
    
    timeout = millis();
    while (digitalRead(clk_pin)) {
      if (millis() - timeout > 50) {
        interrupts();
        return false;
      }
    }
    
    data >>= 1;
  }
  
  // Parity bit
  if (parity) {
    pinMode(data_pin, INPUT_PULLUP);
  } else {
    pinMode(data_pin, OUTPUT);
    digitalWrite(data_pin, LOW);
  }
  
  timeout = millis();
  while (!digitalRead(clk_pin)) {
    if (millis() - timeout > 50) {
      interrupts();
      return false;
    }
  }
  
  timeout = millis();
  while (digitalRead(clk_pin)) {
    if (millis() - timeout > 50) {
      interrupts();
      return false;
    }
  }
  
  // Stop bit
  pinMode(data_pin, INPUT_PULLUP);
  
  timeout = millis();
  while (!digitalRead(clk_pin)) {
    if (millis() - timeout > 50) {
      interrupts();
      return false;
    }
  }
  
  timeout = millis();
  while (digitalRead(clk_pin)) {
    if (millis() - timeout > 50) {
      interrupts();
      return false;
    }
  }
  
  // ACK
  timeout = millis();
  while (digitalRead(data_pin)) {
    if (millis() - timeout > 50) {
      interrupts();
      return false;
    }
  }
  
  timeout = millis();
  while (!digitalRead(clk_pin)) {
    if (millis() - timeout > 50) {
      interrupts();
      return false;
    }
  }
  
  timeout = millis();
  while (digitalRead(clk_pin)) {
    if (millis() - timeout > 50) {
      interrupts();
      return false;
    }
  }
  
  interrupts();
  delayMicroseconds(50);
  return true;
}

void ps2_write(uint8_t clk_pin, uint8_t data_pin, uint8_t data) {
  uint8_t parity = 1;
  
  noInterrupts();
  
  // Request to send
  pinMode(clk_pin, OUTPUT);
  digitalWrite(clk_pin, LOW);
  delayMicroseconds(100);
  
  pinMode(data_pin, OUTPUT);
  digitalWrite(data_pin, LOW);
  
  pinMode(clk_pin, INPUT_PULLUP);
  
  // Wait for device to take control
  while (digitalRead(clk_pin));
  
  // Send data bits
  for (uint8_t i = 0; i < 8; i++) {
    if (data & 1) {
      pinMode(data_pin, INPUT_PULLUP);
      parity ^= 1;
    } else {
      pinMode(data_pin, OUTPUT);
      digitalWrite(data_pin, LOW);
    }
    
    while (!digitalRead(clk_pin));
    while (digitalRead(clk_pin));
    
    data >>= 1;
  }
  
  // Parity bit
  if (parity) {
    pinMode(data_pin, INPUT_PULLUP);
  } else {
    pinMode(data_pin, OUTPUT);
    digitalWrite(data_pin, LOW);
  }
  
  while (!digitalRead(clk_pin));
  while (digitalRead(clk_pin));
  
  // Stop bit
  pinMode(data_pin, INPUT_PULLUP);
  while (!digitalRead(clk_pin));
  while (digitalRead(clk_pin));
  
  // ACK
  while (digitalRead(data_pin));
  while (!digitalRead(clk_pin));
  while (digitalRead(clk_pin));
  
  interrupts();
  delayMicroseconds(50);
}

uint8_t ps2_read_byte(uint8_t clk_pin, uint8_t data_pin) {
  uint8_t data = 0;
  uint8_t bit = 0;
  
  pinMode(clk_pin, INPUT_PULLUP);
  pinMode(data_pin, INPUT_PULLUP);
  
  unsigned long timeout = millis();
  
  while (bit < 11) {
    while (digitalRead(clk_pin)) {
      if (millis() - timeout > 50) return 0;
    }
    
    if (bit > 0 && bit < 9) {
      data >>= 1;
      if (digitalRead(data_pin)) data |= 0x80;
    }
    
    while (!digitalRead(clk_pin)) {
      if (millis() - timeout > 50) return 0;
    }
    
    bit++;
  }
  
  return data >> 1;
}

//=== ADB PROTOCOL ===

void init_adb() {
  pinMode(ADB_PIN, INPUT);
  digitalWrite(ADB_PIN, HIGH);
  delayMicroseconds(3000); // Power-on reset
  
  Serial.println(F("ADB initialized"));
}

void check_adb_attention() {
  // Check for attention signal from Mac
  if (digitalRead(ADB_PIN) == LOW) {
    unsigned long low_time = 0;
    while (digitalRead(ADB_PIN) == LOW && low_time < 1000) {
      delayMicroseconds(1);
      low_time++;
    }
    
    if (low_time >= 800) {
      // Valid attention signal
      delayMicroseconds(200); // Sync time
      
      uint8_t command = adb_receive_byte();
      handle_adb_command(command);
    }
  }
}

void handle_adb_command(uint8_t cmd) {
  uint8_t addr = (cmd >> 4) & 0x0F;
  uint8_t reg = cmd & 0x03;
  uint8_t type = (cmd >> 2) & 0x03;
  
  Serial.print(F("ADB cmd: addr="));
  Serial.print(addr);
  Serial.print(F(" type="));
  Serial.print(type);
  Serial.print(F(" reg="));
  Serial.println(reg);
  
  if (type == 3) { // Talk
    if (addr == 2 && reg == 0) { // Keyboard data
      adb_send_bytes(adb_kbd_data, 2);
      adb_kbd_data[0] = 0xFF;
      adb_kbd_data[1] = 0xFF;
    } else if (addr == 3 && reg == 0) { // Mouse data
      adb_send_bytes(adb_mouse_data, 2);
      adb_mouse_data[0] = 0x80;
      adb_mouse_data[1] = 0x80;
    }
  }
}

void adb_send_bytes(uint8_t* data, uint8_t len) {
  // Start-to-start time
  delayMicroseconds(200);
  
  // Send start bit (1)
  adb_send_bit(1);
  
  // Send data bytes
  for (uint8_t i = 0; i < len; i++) {
    for (int8_t bit = 7; bit >= 0; bit--) {
      adb_send_bit((data[i] >> bit) & 1);
    }
  }
  
  // Send stop bit (0)
  adb_send_bit(0);
  
  // Release bus
  pinMode(ADB_PIN, INPUT);
}

void adb_send_bit(uint8_t bit) {
  pinMode(ADB_PIN, OUTPUT);
  digitalWrite(ADB_PIN, LOW);
  
  if (bit) {
    delayMicroseconds(ADB_BIT1_LOW);
  } else {
    delayMicroseconds(ADB_BIT0_LOW);
  }
  
  pinMode(ADB_PIN, INPUT);
  delayMicroseconds(ADB_BIT_CELL - (bit ? ADB_BIT1_LOW : ADB_BIT0_LOW));
}

uint8_t adb_receive_byte() {
  uint8_t data = 0;
  
  // Wait for start bit
  unsigned long timeout = micros();
  while (digitalRead(ADB_PIN) == HIGH) {
    if (micros() - timeout > 300) return 0;
  }
  
  // Read 8 bits
  for (uint8_t i = 0; i < 8; i++) {
    data <<= 1;
    
    timeout = micros();
    while (digitalRead(ADB_PIN) == LOW) {
      if (micros() - timeout > 100) return 0;
    }
    
    unsigned long high_time = micros();
    while (digitalRead(ADB_PIN) == HIGH) {
      if (micros() - high_time > 100) break;
    }
    
    if (micros() - high_time > 40) {
      data |= 1;
    }
  }
  
  return data;
}
