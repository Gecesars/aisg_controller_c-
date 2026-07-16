# Hardware de referência

## Barramento AISG

O STM32 trabalha em 3,3 V e não deve ser ligado diretamente ao par AISG.
Entre USART1 e o conector, o produto precisa incluir:

- transceptor RS-485 half-duplex, com `DE` e `/RE` unidos em PB12;
- isolamento galvânico com tensão e distâncias adequadas ao produto;
- TVS, proteção contra ESD/surto e proteção de modo comum;
- terminação e polarização dimensionadas para a topologia do sistema;
- conversão isolada da alimentação AISG de 10–30 V para as tensões locais;
- retorno do estado de falha/fonte, se exigido pela análise de segurança.

O primário, e não o RET secundário, aplica o sinal ISB. A entrada do produto
deve tolerar a presença desse sinal conforme TS 25.461.

O callback de transmissão eleva `DE` antes do primeiro start bit, aguarda o
flag `TC` do USART e baixa `DE` imediatamente após o stop bit final. O
protocolo agenda a resposta no mínimo 3 ms após o flag final recebido e antes
do limite de 10 ms.

## Atuador

O backend de referência espera um DRV8833:

- PC7 em IN1 e PC6 em IN2;
- PB8 em nSLEEP;
- PB9 em nFAULT com pull-up;
- PB10 e PB11 nos fins de curso mínimo/máximo, ativos em nível baixo;
- PA0 em um sensor analógico de posição entre 0 e 3,3 V.

Os fins de curso são necessários para a calibração completa. O potenciômetro
é necessário para posição fechada e detecção de motor travado. O software
rejeita uma calibração cujo vão ADC seja menor que 1.000 contagens.

Se o sentido físico do motor estiver invertido, troque IN1/IN2 ou ajuste
`motor_drive()`. Não inverta apenas os fios dos fins de curso.

## Flash

O setor 11 (128 KiB, endereço `0x080E0000`) é reservado. Registros são
acrescentados sequencialmente com:

1. cabeçalho e número de sequência;
2. configuração RET e calibração ADC;
3. CRC-32 do payload;
4. marcador de commit gravado por último.

Uma queda de energia durante a gravação mantém o último registro válido. O
setor é apagado somente quando não há outro slot livre.

## Checklist de bancada

1. Verificar HSE de 12 MHz e erro real do baud rate em 9.600 bit/s.
2. Medir 8N1 e o tempo entre flag recebido e primeiro start bit transmitido.
3. Medir a retirada de `DE` após o último stop bit; o limite é 20 tempos de
   bit.
4. Ensaiar alimentação, consumo, isolação e proteção segundo TS 25.461.
5. Calibrar o curso e validar erro/overshoot de tilt em frio, quente e carga.
6. Simular nFAULT, sensor aberto, motor bloqueado e fim de curso incoerente.
7. Executar varredura e procedimentos com pelo menos dois primários AISG 2.0.
8. Executar os casos oficiais de conformidade aplicáveis ao produto.
