# ps2adb
## Manual do Programador

Este manual destina-se a quem quer modificar ou estender o código para converter PS/2 teclado + rato em ADB para Macintosh clássicos. Foca-se na arquitetura, pontos de extensão, parâmetros e boas práticas de timing em microcontroladores AVR (Arduino 1.8.x).

### 1. Visão Geral da Arquitetura

Componentes principais:

- Dois canais de entrada PS/2 independentes (teclado e rato). Cada canal tem Clock e Data dedicados.
- Um barramento ADB (linha única open-collector, com pull-up externo) partilhado para teclado e rato emulados (endereços ADB distintos).
- Conversor de scan codes PS/2 Set 2 → códigos ADB (teclado). Tabela parcial para letras, dígitos, pontuação, modificadores e navegação.
- Empacotador de movimento do rato PS/2 (3 bytes) para o formato ADB Handler 1/2 (2 bytes) com bits de botão e deltas de movimento truncados.
- Mecanismo bidirecional: suporta Talk (Mac → solicitar dados) e Listen (Mac → enviar comandos, ex. LEDs do teclado) em ADB.
- Inibição coordenada de linhas PS/2 durante janelas críticas ADB para minimizar jitter de timing.

Fluxo de dados (simplificado):

1. Poll não-bloqueante leitura PS/2 → geração de eventos (teclado) e deltas (rato).
2. Marca pendências (`kbdpending`, `mousepending`).
3. Mac envia comando ADB (Talk). Código detecta atenção/start bit e desenrola receção do byte de comando.
4. Se Talk reg0: envia pacote de evento (teclado) ou movimento (rato).
5. Se Listen reg2 (teclado): recebe byte de estado LEDs e emite sequência PS/2 (0xED + máscara).

### 2. Ficheiros e Responsabilidades

- `llm01/llm01.ino`: Sketch principal com toda a lógica ADB/PS/2 parametrizada.
- `adbduino/adbduino.ino`: Referência original funcional para rato; fonte das rotinas de temporização ADB.
- `adbduino/ps2kbd.h`: Implementação mais completa de comandos PS/2 (reset, repeat, LEDs) usada como inspiração.

### 3. Parâmetros Configuráveis (Defines)

No topo de `llm01.ino`:

- `SERIAL_LOG` (1/0) – Ativa ou desativa logging via `Serial`.
- `PS2K_CLK`, `PS2K_DAT`, `PS2M_CLK`, `PS2M_DAT` – Pinos PS/2 de teclado e rato.
- `ADB_PIN` – Pino físico da linha ADB (deve ligar a transistor open-collector + pull-up ~4.7k–10k).
- `LED_PIN` – LED de atividade.
- `PS2_B0_LEFT_ACTIVE_HIGH` – Se 1, bit0=1 indica botão esquerdo premido; se 0, inverte.
- `MOUSE_DELTA_CLAMP` – Limite máximo absoluto (±) antes de truncar a 6 bits (padrão 63).
- `ADB_ADDR_KBD`, `ADB_ADDR_MSE` – Endereços ADB dos dispositivos emulados (teclado=2, rato=3 típicos).

Agora existe um ficheiro `llm01/config.h` que centraliza todas as opções. Podes editar diretamente lá (ou definir via build antes de incluí-lo).

Opções principais:

- Logging: `SERIAL_LOG` (1/0)
- Pinos: `PS2K_CLK`, `PS2K_DAT`, `PS2M_CLK`, `PS2M_DAT`, `ADB_PIN`, `LED_PIN`
- Endereços: `ADB_ADDR_KBD`, `ADB_ADDR_MSE`
- Rato: `PS2_B0_LEFT_ACTIVE_HIGH` (polaridade do botão esquerdo), `MOUSE_DELTA_CLAMP`
- PS/2 robustez: `PS2_REQUIRE_ACK` (requer ACK nos comandos host→teclado, p.ex. LEDs)
- Setup do rato: `PS2M_SETUP_ENABLE`, `PS2M_SAMPLE_RATE`, `PS2M_RESOLUTION`
- Handler do rato: `ADB_MOUSE_HANDLER` (2=dois bytes, 4=stub de três bytes para botão direito no terceiro byte)

### 4. Estrutura de Estados Internos

- Teclado:
	- `kbd_fifo[8]` – Buffer circular de eventos ADB (até 8 transições).
	- `kbdpending` – Indica que há um pacote pronto para envio em Talk reg0.
	- `kbdskip`/`kbdprev0` – Tratamento de anomalia (código 0xFF) e fallback.
	- Flags: `mod_shift`, `mod_ctrl`, `mod_alt`, `mod_cmd` (para sintetizar reg2).

- Rato:
	- `mousepending` – Sinaliza que `mousereg0` contém movimento/botão para enviar.
	- `mousereg0` – Pacote ADB de 2 bytes pronto a ser enviado.
	- `mouse_dx`, `mouse_dy` – Deltas acumulados (debug/estatística).

- ADB:
	- `kbdsrq`, `mousesrq` – Service Request (SRQ) para indicar ao host que deve “falar” (Talk).
	- Temporização bitcell implementada por `place_bit0`, `place_bit1` e `send_byte`.

### 5. Temporização ADB (Critérios)

Bit cell ~100 µs (0: 65 LOW + 35 HIGH; 1: 35 LOW + 65 HIGH). Tolerância do device é mais larga, mas recomenda-se manter proporção.

Funções:
- `place_bit0()` / `place_bit1()` – Geram padrões LOW/HIGH.
- `adb_recv_cmd()` – Reconhecimento de atenção + receção de comando (8 bits).
- `adb_recv_data_byte()` – Recebe byte subsequente em Listen.

Evita chamadas prolongadas / bloqueantes entre transições para não perder janelas de host.

### 6. Inibição PS/2 Coordenada

Funções:
- `ps2_inhibit_all()` – Força Clock do teclado e rato a LOW (inibe tráfego PS/2).
- `ps2_release_all()` – Liberta ambos (INPUT_PULLUP).

Aplicadas antes e depois de envio ADB crítico (Talk reg0/2 e Listen reg2). Reduz risco de colisões de timing.

### 7. Extensões Recomendadas

1. Setup completo do rato (reset, sample rate, scaling) usando sequência similar à de `ps2kbd.h`.
2. Suporte a registos adicionais ADB (ex. Talk reg3 para re-endereçar; Handler IDs).
3. Modo “diagnóstico”: imprime raw timing e duty dos bits recebidos (útil para calibrar em máquinas sensíveis).
4. Buffer maior para teclado (se pretender taxas elevadas). Atenção ao impacto em SRAM.
5. Abstrair drivers PS/2 para classe separada (facilita porte para outras arquiteturas que não AVR).

### 8. Como Adicionar Novo Mapeamento de Tecla

1. Identifica o scan code Set 2 (ex.: usando logs ou tabela OSDev).
2. Localiza a tabela `map_set2_to_adb[]`.
3. Acrescenta entrada `{0xSCANCODE, 0xADB}` onde 0xADB é o código de key Mac clássico.
4. Se for extended (prefixo 0xE0), adiciona/manipula em `ps2e0_to_adb()`.
5. Testa pressionar e largar; confirma no log `[KBD] DN/UP`.

### 9. Como Alterar Tratamento do Rato

Clamps e inversão:
- Ajusta `MOUSE_DELTA_CLAMP` se movimento estiver “saltitante”. Valores menores reduzem jitter.
- Ajusta `PS2_B0_LEFT_ACTIVE_HIGH` se botão esquerdo estiver lido ao contrário.

Para incluir botão direito no pacote ADB mais rico (3 bytes – Handler 4), seria necessário:
1. Define `ADB_MOUSE_HANDLER 4` em `llm01/config.h` (stub ativado): o terceiro byte envia o botão direito no bit7 (resto 0). Validar se o host aceita 3 bytes; caso contrário, manter em 2.
2. Se precisares de deltas de 7 bits cheios no Handler 4, será necessário gerar os 3 bytes completos conforme especificação — o stub atual só sinaliza o botão direito.
3. A sinalização de SRQ mantém-se inalterada no stub; adaptações adicionais podem ser necessárias num suporte completo de Handler 4.

### 10. Debug e Troubleshooting

Sintomas comuns:
- “Rato não mexe”: verificar transistor/open-collector, pull-up na linha ADB (~4.7k). Confirmar que Mac envia Talk (usar lógica para ver a linha pulsar).
- Teclas não chegam ao Mac: ver se `kbdpending` sobe e se o host faz Talk reg0 (pode estar preso esperando SRQ → conferir `kbdsrq`).
- LEDs não atualizam: confirmar que o Mac envia Listen reg2 (dump de comando via log). Checar sequência 0xED + máscara no osciloscópio.
- Jitter excessivo: reduzir `SERIAL_LOG` (log consome tempo), diminuir clamp, ativar inibição PS/2 (já implementada).

Ferramentas:
- Serial log a 115200 para eventos.
- Osciloscópio ou lógica: mede bitcell ADB (~100 µs), verifica duty de “0” e “1”.

### 11. Boas Práticas de Contribuição

- Manter mudanças de timing confinadas às funções `place_bit0/1`.
- Evitar delays longos dentro do loop principal.
- Parametrizar novos comportamentos via `#define` para não quebrar setups existentes.
- Documentar cada novo ADB register suportado com referência a fonte (datasheet, wiki, código legado).
- Testar em hardware real (emuladores de ADB podem ser mais permissivos).

### 12. Estrutura do Loop Principal

Pseudo-código:
```
loop:
	poll_ps2_keyboard()
	poll_ps2_mouse()
	preparar kbdpending se FIFO tem evento
	se mousepending || kbdpending: ler cmd ADB
	se cmd destinatário rato e Talk reg0: enviar mousereg0
	se cmd destinatário teclado e Talk reg0: enviar kbdreg0
	se cmd Listen reg2 teclado: aplicar LEDs PS/2
	piscar LED atividade
```

### 13. Segurança e Estabilidade

- Evitar saturar interrupções: o sketch não define ISR próprio; timing depende de busy-wait curto.
- Landings futuros podem mover bit-banging para Timer/ISR para jitter menor (cuidado com latência do Serial).
- Sempre usar fonte estável de 5V (o ADB clássico espera aproximação a 5V através de pull-up).

### 14. Referências (Relembradas)

- Microchip AN591B – Apple Desktop Bus.
- “ADB: An Introduction” – artigo técnico.
- Linux `adbmouse.c` – estrutura de pacotes de movimento.
- Tabelas PS/2 Set 2 – OSDev Wiki.
- Lista de keycodes Mac clássicos – várias fontes (virtual keycodes Mac).

### 15. Roadmap Sugerido

- [ ] Implementar setup de rato (reset, sample rate configurável via define).
- [ ] Handler 4 para terceiro byte (botão direito).
- [ ] Modo diagnóstico com medição real de low/high para cada bit recebido.
- [ ] Abstrair em classes (KeyboardPS2, MousePS2, ADBDevice) para reutilização em outros MCUs.
- [ ] Testes automatizados básicos com mocks de linha (simulação de bitcell).

---
Este manual deve servir como referência viva: atualiza sempre que novos registos ADB, macros ou fluxos forem introduzidos.

Collection of projects to hopefully have this thing working


## adbduino

code from a human

## llm01

code from chatgpt
