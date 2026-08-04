# Contrato da IHM COMUNICACAO-MOTOR

A fonte canônica está em `../../docs/protocol_contract.md`.

Esta IHM exige protocolo `0xC002` e dispositivo `0xF301`. Motor, PWM e rotinas são comandados pelo Modbus, mas executados localmente no STM32. Nenhum diagnóstico ADC é consultado e o único erro aceito é E08.
