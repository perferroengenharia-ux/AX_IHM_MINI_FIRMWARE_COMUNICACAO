# AX_IHM_MINI_FIRMWARE_COMUNICACAO

Firmware ESP32-S3 mestre Modbus da variante `COMUNICACAO-SENSORES-1.2.0`. Ele aceita somente STM32 com protocolo `0xC003` e dispositivo `0xF301`.

## Funções

- RS485 em GPIO37 TX, GPIO36 RX e GPIO38 RTS/DE, 9600 bit/s, 8E1;
- identidade, sincronização de parâmetros, heartbeat e recuperação de E08;
- comandos de motor, frequência, sentido, rampa, torque e portadora;
- comandos de bomba, swing e diagnóstico do sensor de nível;
- leitura periódica e cache atômico de barramento, corrente, temperatura e falhas elétricas;
- comandos de diagnóstico ADC/OPAMP, simulação e reset seguro de falhas;
- rotinas de molhagem, secagem e exaustão executadas no STM32;
- parâmetros persistidos em NVS;
- terminal USB/UART a 115200 bit/s.

O STM32 mede barramento, corrente e temperatura, protege por E02–E06 e mantém E08 para comunicação. PA11/SD-OD usa TIM1_BKIN2; PB11 e PB12 não são usados. O BYPASS/PA15 permanece alto sem falha e é desligado por falha elétrica ativa.

## Compilar e testar

```powershell
pio run
.\test\host\run_tests.ps1
```

## Uso rápido

```text
motor freq 10
motor start
motor up 5
motor down 2
motor status
pwm status
motor stop

pwm freq 10
ramp accel 15
ramp decel 10
torque gain 4
sensor status
sensor raw
error status
```

Configuração e execução da secagem:

```text
param unlock
param set P32 3000
param set P33 1
param set P31 3
param set P86 1
param lock
routine dry start
routine status
```

Molhagem:

```text
param unlock
param set P30 2
param lock
routine wet start
routine status
routine stop
```

Use `help` para a lista completa. O contrato detalhado está em `../docs/protocol_contract.md` no projeto STM32.
