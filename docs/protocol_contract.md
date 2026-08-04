# Contrato da IHM COMUNICACAO

A fonte canônica do contrato está em `../../docs/protocol_contract.md`.

Esta IHM exige versão Modbus `0xC001` e dispositivo `0xF301`. Ela consulta estado de comunicação, bomba, swing e sensor de nível. Não consulta diagnósticos ADC nem envia comandos de simulação, motor ou PWM.
