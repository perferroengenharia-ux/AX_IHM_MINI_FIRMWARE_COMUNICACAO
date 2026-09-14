# AX_IHM_MINI_FIRMWARE_COMUNICACAO

Firmware ESP32-S3 mestre Modbus da variante `COMUNICACAO-SENSORES-MQTT-1.3.3`. Ele aceita somente STM32 com protocolo `0xC003` e dispositivo `0xF301`.

A versao 1.3.3 preserva o transporte e o mapa Modbus validados e acrescenta
somente extensoes opcionais para a area de assistencia tecnica do aplicativo:
telemetria eletrica, diagnostico IHM-MI, snapshot P10..P91 e os comandos MQTT
`request-parameters`/`set-parameter`. A escrita remota passa pelo mesmo servico,
validacao, NVS e handshake usados por `param set` no terminal.

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
- AP local `AXON-IHM-SETUP`, aberto, em `192.168.4.1:8080`;
- provisionamento de Wi-Fi pelos endpoints usados pelo aplicativo;
- MQTT TLS com certificado validado e contrato `axon.ihm.v1`;
- estado de motor, bomba, swing, nível, leituras elétricas e falhas no app;
- comandos do app encaminhados pela fila Modbus validada, sem acesso paralelo à UART.

## Aplicativo, AP e MQTT

Na primeira configuração, conecte o telefone ao AP `AXON-IHM-SETUP` e use o
provisionamento do aplicativo. O firmware mantém o AP ativo também durante a
conexão STA, então a API local continua disponível em
`http://192.168.4.1:8080/api/v1`.

O MQTT usa os tópicos `axon/ihm/<deviceId>/status`, `state`, `capabilities`,
`commands`, `events`, `errors` e `schedules`. Status, estado, capacidades e
agendamentos são retidos; comandos e confirmações não são. O firmware publica
um snapshot operacional a cada 5 s e o snapshot completo a cada 60 s. As
consultas HTTP leem somente o cache existente e não acrescentam tráfego RS485.

Comandos aceitos pelo app: ligar/desligar com as etapas normais de molhagem e
secagem (ou `skip-stage`), alterar frequência, bomba, swing, iniciar/parar a
rotina de secagem/exaustão, solicitar status/capacidades e sincronizar a lista
de agendamentos.

Na partida normal, o STM32 liga o motor automaticamente ao concluir P30 e
mantém a bomba ligada enquanto o sensor permitir. Na parada normal, a bomba é
desligada imediatamente e o motor permanece ligado durante P31: P32 diferente
de zero altera o alvo pela rampa; P32 igual a zero mantém a frequência anterior.
Depois de P31 o motor desacelera e o sistema é desligado. Essas sequências são
locais e não geram polling adicional.

A tabela `partitions.csv` reserva 3 MB para o aplicativo na flash física de
8 MB. Esse espaço adicional é necessário para Wi-Fi, TLS, HTTP e MQTT; a NVS
continua separada para preservar parâmetros, credenciais e agendamentos.

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
