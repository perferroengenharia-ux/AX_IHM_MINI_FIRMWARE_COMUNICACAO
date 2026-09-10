# Contrato da IHM COMUNICACAO-SENSORES

A fonte canônica está em `../../docs/protocol_contract.md`.

Esta IHM exige protocolo `0xC003` e dispositivo `0xF301`. Motor, PWM e rotinas são comandados pelo Modbus, mas executados localmente no STM32. A IHM consulta o snapshot atômico de barramento, corrente, temperatura e falhas, mantém cache local e oferece `sensor status`, `sensor raw` e `error status|reset|clear-e08`.
