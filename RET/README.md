# Firmware RET AISG 2.0 para STM32F4

Implementação em C11 de um dispositivo secundário RET (Remote Electrical
Tilt) AISG 2.0 para STM32F405RG/Pyboard clássica. O arquivo
`versao_final_stm32F4.py` foi usado como referência funcional e de pinagem;
o protocolo foi reimplementado a partir das especificações, sem depender do
MicroPython.

## O que está implementado

- enlace serial assíncrono a 9.600 bit/s, 8N1 e controle de direção RS-485;
- enquadramento HDLC, transparência `0x7D`, FCS CRC-16/X-25 e recepção
  incremental sem alocação dinâmica;
- estados `NoAddress`, `AddressAssigned` e `Connected`;
- XID para varredura, atribuição de endereço, negociação, versão AISG 2.0 e
  reset;
- janela HDLC igual a 1, I/RR/RNR/SNRM/DISC/UA/DM/FRMR, retransmissão,
  detecção de duplicatas e timeout de três minutos;
- conjunto obrigatório comum RETAP: Reset, alarmes, informações, dados do
  usuário, assinatura de alarmes e autoteste;
- conjunto obrigatório sRET: calibração, configuração segmentada, ajuste e
  leitura de tilt, dados e indicação de alarmes;
- conjunto obrigatório mRET no núcleo: procedimentos `0x80` a `0x89`;
- procedimentos demorados assíncronos, sem bloquear o enlace;
- controle DRV8833, dois fins de curso, potenciômetro de posição, calibração
  de curso completo e detecção de motor/atuador travado;
- persistência em journal com CRC e marcador de commit no setor 11 da flash;
- testes nativos do protocolo, inclusive com sanitizadores.

O firmware de placa fornecido é sRET: há um único conjunto físico de motor e
sensor. O núcleo suporta mRET, mas uma placa mRET precisa fornecer um backend
de motor por antena.

## Estrutura

```text
RET/
├── include/ret/              API pública do protocolo
├── src/                      HDLC e RETAP independentes de hardware
├── platform/stm32f4/         HAL, pinagem, motor, flash, startup e linker
├── platform/host/            adaptador POSIX para bancada virtual
├── tests/                    testes executados no computador
├── docs/                     hardware e matriz de conformidade
└── cmake/                    toolchain GNU Arm Embedded
```

## Pinagem padrão

| Função | CPU | Pyboard | Direção |
|---|---:|---:|---:|
| USART1 TX | PB6 | X9 | saída |
| USART1 RX | PB7 | X10 | entrada |
| RS-485 DE + /RE | PB12 | Y5 | saída |
| DRV8833 IN2 | PC6 | Y1 | saída |
| DRV8833 IN1 | PC7 | Y2 | saída |
| DRV8833 nSLEEP | PB8 | Y3 | saída |
| DRV8833 nFAULT | PB9 | Y4 | entrada, ativo baixo |
| fim de curso mínimo | PB10 | Y9 | entrada, ativo baixo |
| fim de curso máximo | PB11 | Y10 | entrada, ativo baixo |
| posição analógica | PA0/ADC1_IN0 | X1 | entrada 0–3,3 V |

Toda a pinagem pode ser alterada em
`platform/stm32f4/ret_board_config.h`.

Não conecte o par AISG diretamente ao STM32. Use um transceptor RS-485
half-duplex isolado, protegido contra surto e adequado à instalação. O
condicionamento de 10–30 V, isolamento, bias e terminação fazem parte do
hardware, não do firmware. Veja [docs/HARDWARE.md](docs/HARDWARE.md).

## Testar no Linux

```bash
cmake -S RET -B RET/build-host -DRET_BUILD_STM32=OFF
cmake --build RET/build-host --parallel
ctest --test-dir RET/build-host --output-on-failure
```

O mesmo build também produz `RET/build-host/ret_host`. Ele executa o `ret_core`
do firmware em uma porta serial POSIX, substituindo somente os periféricos. O
exemplo abaixo cria dois RETs em `NoAddress`, com UIDs distintos, no mesmo
barramento:

```bash
RET/build-host/ret_host --port /tmp/aisg_ret --devices 2
```

Normalmente não é necessário chamá-lo diretamente: o utilitário
`simulador_serial` compila e inicia esse executável automaticamente. Para um
teste legado com endereços já atribuídos, acrescente `--address 1`; as instâncias
receberão endereços sequenciais.

No backend POSIX, um comando `Set Tilt` simula o deslocamento por
`|alvo - posição atual| × 2 segundos por grau`. O procedimento permanece
assíncrono no mesmo `ret_core` do STM32: o primeiro retorno e cada consulta
RR/P recebem RR enquanto o motor está em movimento, e a resposta RETAP de
sucesso só é transmitida depois que o tempo de deslocamento termina. Calibração
e autoteste conservam o atraso determinístico curto de bancada. No STM32 real,
o término continua sendo definido pelo sensor e pelo backend físico do motor,
não por esse relógio artificial.

Uma execução com ASan/UBSan pode ser criada assim:

```bash
cmake -S RET -B RET/build-sanitize \
  -DCMAKE_C_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined"
cmake --build RET/build-sanitize --parallel
ctest --test-dir RET/build-sanitize --output-on-failure
```

## Compilar o firmware STM32F405

Pré-requisitos: GNU Arm Embedded (`arm-none-eabi-gcc`) e uma cópia do
[STM32CubeF4 oficial](https://github.com/STMicroelectronics/STM32CubeF4).

```bash
cmake -S RET -B RET/build-stm32 \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/RET/cmake/arm-none-eabi-gcc.cmake" \
  -DRET_BUILD_STM32=ON \
  -DRET_BUILD_TESTS=OFF \
  -DSTM32_CUBE_F4_PATH=/caminho/para/STM32CubeF4
cmake --build RET/build-stm32 --parallel
```

São produzidos `ret_stm32f405.elf`, `.hex`, `.bin` e o mapa do linker. O
linker reserva `0x080E0000–0x080FFFFF` para persistência; não altere esse
setor sem atualizar a configuração da flash.

Grave por SWD ou pelo bootloader DFU apropriado à sua placa. Confirme o
modelo exato do MCU, a frequência HSE de 12 MHz e o mapa de memória antes de
gravar. A Pyboard D não é STM32F4 e não usa este alvo.

## Identidade e configuração de produção

Os valores visuais/funcionais do código MicroPython foram aproveitados para
produto, modelo, faixa de tilt e fornecedor `TY`. O identificador de unidade
e o serial são derivados do UID de fábrica do STM32 para que duas placas não
respondam com a mesma identidade.

Antes de produção, substitua `TY` pelo código de fornecedor realmente
atribuído à organização e revise todos os dados da antena. Um código não
atribuído impede declarar conformidade formal, mesmo que os frames estejam
corretos.

## Escopo de conformidade

O núcleo cobre os requisitos obrigatórios de protocolo secundário de
[3GPP TS 25.462](https://www.etsi.org/deliver/etsi_ts/125400_125499/125462/06.04.00_60/ts_125462v060400p.pdf)
e o conjunto RET de
[3GPP TS 25.463](https://www.etsi.org/deliver/etsi_ts/125400_125499/125463/06.05.00_60/ts_125463v060500p.pdf),
com as extensões de versão de
[AISG 2.0](https://aisg.org.uk/file/AISG-v2.0.pdf). A camada elétrica segue
como requisito de projeto a
[3GPP TS 25.461](https://www.etsi.org/deliver/etsi_ts/125400_125499/125461/06.05.00_60/ts_125461v060500p.pdf).

“100% compatível” só pode ser afirmado para um produto após ensaio elétrico,
temporização no hardware real e interoperabilidade com um primário AISG. A
matriz objetiva do que é coberto e do que ainda exige bancada está em
[docs/CONFORMIDADE_AISG2.md](docs/CONFORMIDADE_AISG2.md).
