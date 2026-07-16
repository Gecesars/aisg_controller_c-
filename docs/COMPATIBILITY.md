# Compatibilidade AISG Base 3.0.8 e ADB 3.1.7

## Escopo da revisão

O perfil serial AISG 3 foi revisado contra os documentos locais:

- **AISG Base Standard v3.0.8.2**, de 14 de outubro de 2024;
- **AISG-ST-ADB vADB3.1.7.6**, de 27 de junho de 2024.

Os PDFs são entradas locais de validação e não fazem parte do repositório. Na
negociação em barramento são usados os números de versão normativos `3.0.8` e
`3.1.7`; o último componente impresso na capa identifica a revisão editorial do
documento e não é transmitido na tupla `AISGVersion_t`.

Esta matriz descreve um **perfil primário Base/ADB de descoberta e diagnóstico**.
Ela não declara certificação AISG do produto inteiro.

## Matriz de implementação

| Área | Requisito aplicado | Estado |
|---|---|---|
| Camada 1 | 9,6 kbit/s, 8 bits de dados, sem paridade, um stop bit | Implementado em `termios` |
| Camada 1 | RS-485 de dois fios, half-duplex | Requer adaptador externo compatível |
| Camada 1 | Desabilitar o driver até 2 ms após o último stop bit | Requer controle automático de direção do adaptador; não verificável por software genérico |
| HDLC | Flags `0x7E`, escape `0x7D`/XOR `0x20` e CRC-16/X-25 | Implementado e testado |
| HDLC | Quadro sem flags de 4 a 268 octetos; INFO até 264 octetos | Limites aplicados no encoder e decoder |
| HDLC | SNRM `0x93` / UA `0x73`, I-frames com janela 1 | Implementado e testado |
| HDLC | Intervalo mínimo de 3 ms após resposta Final | Aplicado pelo controlador |
| XID | FI `0x81`, GI `0xF0`, GL de um octeto e inteiros big-endian | Implementado e testado |
| XID | Device Scan v1 com PI 1, 8, 10, 11 e 19 | Implementado |
| XID | Resposta com UID completo, tipo, fornecedor, porta, versões Base e tipos de subunidade | Validada estritamente |
| XID | Resolução de colisões no padrão UID + porta de 21 octetos | Implementada bit a bit |
| XID | Confirmação exata antes da atribuição | Implementada |
| XID | Atribuição com PI 2, 22 e PrimaryID PI 26 | Implementada |
| Identidade | UID com 2 octetos de fornecedor + 17 octetos de unidade | Validado, inclusive preenchimento `0x00` permitido |
| Identidade | PrimaryID = 32 bits iniciais de SHA-1 do nome do nó, zero convertido em um | Implementado e testado com vetor SHA-1 conhecido |
| Base L7 | Cabeçalho de quatro `uint16_t`, little-endian, dados até 256 octetos | Implementado e testado |
| Base L7 | `GetInformation` (`0x0005`) | Implementado; strings UTF-8 validadas |
| Base L7 | `GetSubunitList` (`0x0008`) | Implementado |
| Base L7 | Get/Set Subunit Type Standard Version (`0x000D`/`0x000C`) | Implementado para ADB 3.1.7 |
| Base L7 | `GetAlarmStatus` (`0x0004`) e `ClearActiveAlarms` (`0x0006`) | Implementado para controlador e subunidades visíveis |
| ADB | Sete códigos `0x0300` a `0x0306` e formatos de mensagem | Codecs implementados e testados |
| ADB | Proveniência `0..4`; escrita apenas Manual/Automatic | Validada |
| ADB | Modelo/serial UTF-8 até 32 octetos | Validado |
| ADB | Sector ID `TextString_t` até 32 e notas UTF-8 até 140 octetos | Validado |
| ADB | Azimute `0..3599` e tilt mecânico assinado `-900..900`, em décimos de grau | Validado e testado |
| ADB | Até seis RF Path IDs; PrimaryID e inteiros L7 little-endian | Implementado e testado |
| Aplicação | Descoberta, atribuição, inventário e leitura de instalação ADB | Integrado ao modo serial |
| Aplicação | Escrita de instalação ADB e RF Path IDs | Codec disponível; ação real bloqueada por segurança |

## Perfil operacional habilitado

No perfil serial **AISG 3.0.8**, a aplicação:

1. abre a porta em `9600 8N1`;
2. calcula um PrimaryID estável a partir do `/etc/machine-id` usado como nome
   de nó local;
3. procura ALDs no estado `NoAddress` por Device Scan v1;
4. resolve colisões usando o UID e o número da porta;
5. confirma a identidade e seleciona Base `3.0.8`;
6. atribui um endereço entre 1 e 254 e estabelece o link por SNRM/UA;
7. lê informações Base e a lista de subunidades;
8. negocia ADB `3.1.7` após cada atribuição;
9. lê identificação, instalação e alarmes.

O aplicativo não envia automaticamente `ResetPort` em broadcast. Isso evita
derrubar links de outro primário, mas significa que a descoberta encontra apenas
portas no estado `NoAddress`. Em bancada dedicada, reinicie/altere o estado do
ALD de forma controlada antes da descoberta quando necessário.

## Limites conhecidos

Ainda não fazem parte do perfil compatível:

- alimentação AISG, modem OOK, Ping e mapeamento físico do site;
- RR/RNR/FRMR completos, recuperação avançada de link e tráfego assíncrono;
- download/upload de arquivos, firmware e factory reset;
- procedimentos MALD de autoridade, setup e segurança;
- assinatura de alarmes e todas as demais funções obrigatórias de um primário
  AISG Base completo;
- perfis AISG-ST-RET e AISG-ST-TMA v3, inclusive movimento, calibração, ganho e
  self-test no barramento real;
- validação de temporização elétrica, polaridade, terminação e interoperabilidade
  com equipamento físico.

Por esses limites, “compatível” neste projeto significa que os formatos e o
fluxo Base/ADB descritos na matriz são implementados e cobertos por testes. Uma
garantia de interoperabilidade em campo exige ensaio com o adaptador escolhido,
hardware ALD real e, para alegar conformidade/certificação completa, o programa
oficial de testes de interoperabilidade aplicável.

## Evidências automatizadas

`atc_core_tests` contém 12 cenários, incluindo:

- vetores de CRC e quadros HDLC capturados;
- framing, stuffing, streaming e limites máximos;
- ordem, tamanhos e endianess dos parâmetros XID;
- vetor conhecido de SHA-1 para PrimaryID;
- cabeçalhos, respostas, erros, versões, alarmes e limites Base v3;
- todos os comandos ADB, strings, proveniência e grandezas mecânicas;
- um ALD ADB determinístico que executa o fluxo completo do controlador;
- regressão do simulador AISG 2 legado e cancelamento assíncrono.

Execute:

```sh
cmake --build build-release --parallel
ctest --test-dir build-release --output-on-failure
./build-release/atc_core_tests
```
