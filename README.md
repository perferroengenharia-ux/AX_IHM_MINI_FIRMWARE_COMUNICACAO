# AX_IHM_MINI_FIRMWARE_COMUNICACAO

Firmware ESP32-S3 mestre Modbus da variante `COMUNICACAO`. Ele conversa somente com o STM32 que anuncia protocolo `0xC001` e dispositivo `0xF301`.

## Funções

- inicializa RS485 em UART1, GPIO37 TX, GPIO36 RX e GPIO38 RTS/DE;
- valida a identidade do STM32 e sincroniza parâmetros;
- envia heartbeat e trata E08 por perda de comunicação;
- consulta estado da bomba, swing e sensor de nível;
- envia comandos digitais de bomba e swing;
- oferece terminal USB/UART em 115200 bit/s.

Não há leitura de ADC, tensão, corrente ou temperatura, nem comandos de motor/PWM. Campos analógicos legados recebidos do STM32 são ignorados e permanecem zero.

## Compilar

```powershell
pio run
```

## Testes de protocolo

```powershell
.\test\host\run_tests.ps1
```

## Terminal

Use `help` para listar todos os comandos. Os principais são `status`, `comm ping`, `bomba on|off|status`, `swing on|off|status`, `sensor status` e `outputs status`.

O comando `bomba status` informa também o motivo de bloqueio. Se `bomba on`
for recusado pelo STM32, o terminal mostra `PUMP_BLOCKED` com a causa exata,
como `level_not_stable` ou `water_shortage`.

Para a bomba ser liberada, P82 deve estar habilitado e o nível precisa ficar
estável por 5 segundos. Em P85=1, PF0 baixo significa água disponível; em
P85=2, PF0 alto significa água disponível. Use `sensor status` para conferir.

Consulte `docs/protocol_contract.md` para o mapa de registradores.
