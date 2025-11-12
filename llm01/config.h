// Configuração parametrizável do projeto PS/2 → ADB

#ifndef CONFIG_H
#define CONFIG_H

// Logs
#ifndef SERIAL_LOG
#define SERIAL_LOG 1
#endif

// Pinos PS/2 e ADB
#ifndef PS2K_CLK
  #define PS2K_CLK 3
#endif
#ifndef PS2K_DAT
  #define PS2K_DAT 2
#endif
#ifndef PS2M_CLK
  #define PS2M_CLK 7
#endif
#ifndef PS2M_DAT
  #define PS2M_DAT 6
#endif
#ifndef ADB_PIN
  #define ADB_PIN 12
#endif
#ifndef LED_PIN
  #define LED_PIN 13
#endif

// Endereços ADB
#ifndef ADB_ADDR_KBD
  #define ADB_ADDR_KBD 2
#endif
#ifndef ADB_ADDR_MSE
  #define ADB_ADDR_MSE 3
#endif

// Rato: comportamento
#ifndef PS2_B0_LEFT_ACTIVE_HIGH
  #define PS2_B0_LEFT_ACTIVE_HIGH 1
#endif
#ifndef MOUSE_DELTA_CLAMP
  #define MOUSE_DELTA_CLAMP 63
#endif

// PS/2: robustez de comandos host→device
#ifndef PS2_REQUIRE_ACK
  #define PS2_REQUIRE_ACK 0
#endif

// Setup do rato PS/2
#ifndef PS2M_SETUP_ENABLE
  #define PS2M_SETUP_ENABLE 1
#endif
#ifndef PS2M_SAMPLE_RATE
  #define PS2M_SAMPLE_RATE 0x64
#endif
#ifndef PS2M_RESOLUTION
  #define PS2M_RESOLUTION 0x02
#endif

// Futuro: Handler 2 (2 bytes) ou 4 (3 bytes)
#ifndef ADB_MOUSE_HANDLER
  #define ADB_MOUSE_HANDLER 2
#endif

#endif // CONFIG_H
