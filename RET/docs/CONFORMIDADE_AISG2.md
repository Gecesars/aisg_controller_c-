# Matriz de conformidade AISG 2.0

Esta matriz trata o firmware como dispositivo secundário RET. “Implementado”
significa coberto pelo código e pelos testes locais; não substitui ensaio de
conformidade do produto final.

## Camada 1 — TS 25.461

| Requisito | Estado | Evidência/observação |
|---|---|---|
| Serial assíncrona 9.600 bit/s, 8N1 | Implementado | Inicialização USART1 na plataforma STM32 |
| RS-485 half-duplex | Implementado em software | `DE` explícito; exige transceptor externo |
| Retirada do driver em até 20 bits | Projetado | baixa `DE` após `UART_FLAG_TC`; medir no hardware |
| Alimentação 10–30 V e limites de consumo | Hardware pendente | exige fonte e ensaio do produto |
| Isolação, EMC, surto e ISB | Hardware pendente | não demonstrável por código |

## Camada 2 — TS 25.462 + AISG 2.0

| Requisito | Estado |
|---|---|
| HDLC UNC1/15.1 TWA, flags e transparência básica | Implementado/testado |
| CRC-16/X-25, FCS little-endian | Implementado/testado com vetor conhecido |
| Payload padrão 74 octetos e negociação PI 5/6 | Implementado/testado |
| Janela de transmissão/recepção igual a 1 | Implementado/testado |
| Estados NoAddress/AddressAssigned/Connected | Implementado/testado |
| Scan com UID/máscara e padding de 19 octetos | Implementado/testado |
| Atribuição por UID, tipo e fornecedor | Implementado/testado |
| XID Release ID, AISG version e substance version | Implementado/testado |
| Reset XID broadcast/unicast | Implementado |
| SNRM, DISC, I, RR, RNR, UA, DM e FRMR | Implementado |
| Sequência, duplicata, ACK/NAK e retransmissão | Implementado/testado |
| Atraso mínimo de 3 ms e início antes de 10 ms | Implementado; medir na placa |
| Reset após timeout de enlace de 3 minutos | Implementado/testado |
| Tipos sRET `0x01` e mRET `0x11` | Núcleo implementado; placa física sRET |

## Camada 7 RETAP — TS 25.463

| Grupo | Procedimentos | Estado |
|---|---|---|
| Comum obrigatório | `03`, `04`, `05`, `06`, `0A`, `10`, `11`, `12` | Implementado |
| sRET obrigatório | `0E`, `0F`, `31`, `32`, `33`, `34`, indicação `07` | Implementado |
| mRET obrigatório | `80`–`89`, indicação `85` | Implementado no núcleo |
| Dados adicionais | campos `01`–`07` e `21`–`26` | Implementado |
| Alarmes | status, clear, subscribe e mudanças | Implementado |
| TCP | calibração <4 min e set tilt <2 min | Estado assíncrono implementado |
| Configuração segmentada | segmentos no limite negociado | Implementado/testado |
| User data não volátil | offsets e limites | Implementado/testado indiretamente |
| Download de firmware `40`–`42` | Opcional, não suportado | retorna `UnsupportedProcedure`, permitido pela norma |
| Procedimento vendor `90` | Opcional, não suportado | retorna `UnsupportedProcedure` |

## Condições antes de declarar conformidade de produto

- obter e configurar um código de fornecedor AISG válido; `TY` veio do
  protótipo MicroPython e não é afirmado aqui como código atribuído;
- confirmar o modelo exato da Pyboard/STM32F4 e a frequência do cristal;
- terminar e revisar o circuito RS-485/alimentação/proteção;
- medir temporização e tolerância de baud no hardware final;
- validar calibração, precisão, overshoot e falhas do atuador real;
- executar testes de interoperabilidade e o programa de conformidade AISG
  aplicável.

## Referências normativas

- [AISG 2.0](https://aisg.org.uk/file/AISG-v2.0.pdf)
- [3GPP TS 25.461 V6.5.0 — Layer 1](https://www.etsi.org/deliver/etsi_ts/125400_125499/125461/06.05.00_60/ts_125461v060500p.pdf)
- [3GPP TS 25.462 V6.4.0 — Layer 2](https://www.etsi.org/deliver/etsi_ts/125400_125499/125462/06.04.00_60/ts_125462v060400p.pdf)
- [3GPP TS 25.463 V6.5.0 — RET application](https://www.etsi.org/deliver/etsi_ts/125400_125499/125463/06.05.00_60/ts_125463v060500p.pdf)
