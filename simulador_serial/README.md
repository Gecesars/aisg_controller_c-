# Simulador serial AISG 2.0

Esta ferramenta cria duas portas seriais virtuais interligadas no Linux. O
controlador abre uma porta e dois RETs de bancada em C compartilham a outra
automaticamente. Todo byte recebido é encaminhado sem alteração ao lado oposto
e os quadros HDLC/AISG 2.0 completos são mostrados em tempo real no terminal e
gravados em arquivo.

```text
Controlador                 simulador_serial                    RET 1 + RET 2
    │                /tmp/aisg_controlador  /tmp/aisg_ret             │
    └──── 9600 8N1 ────────► [ ponte + monitor ] ◄──── 9600 8N1 ──────┘
                         CONTROLADOR -> RET
                         RET -> CONTROLADOR
```

## Requisitos

- Linux com Python 3.10 ou superior;
- CMake e compilador C para preparar o RET na primeira execução;
- nenhuma biblioteca Python externa;
- ambos os programas devem abrir a porta em 9600 baud, 8 bits, sem paridade e
  1 stop bit (9600 8N1).

As portas são PTYs do kernel, equivalentes a um par *null-modem* virtual. Os
links estáveis em `/tmp` evitam que os nomes `/dev/pts/N` mudem entre execuções.

## Uso rápido

Na raiz do repositório, execute:

```bash
python3 -m simulador_serial.simulador_serial
```

A inicialização mostra os dois destinos, por exemplo:

```text
# porta CONTROLADOR: /tmp/aisg_controlador -> /dev/pts/4
# porta RET: /tmp/aisg_ret -> /dev/pts/5
# ponte 9600 8N1 ativa; pressione Ctrl+C para encerrar
```

No Antenna Tilt Controller:

1. selecione `Serial RS-485 • AISG 2.0`;
2. use a porta `/tmp/aisg_controlador`;
3. conecte e inicie a localização de dispositivos.

Não é necessário iniciar outro processo manualmente: o simulador configura e
compila `RET/build-host/ret_host`, instancia duas vezes o mesmo `ret_core` C
usado pelo alvo STM32 e conecta ambos a `/tmp/aisg_ret`. Apenas a camada de
plataforma é trocada: PTY substitui UART/RS-485, relógio POSIX substitui
`HAL_GetTick`, a flash é mantida em memória e o motor virtual percorre cada grau
em dois segundos.

Os dois RETs iniciam em `NoAddress`, com UIDs distintos. O controlador transmite
o Device Scan XID em broadcast, confirma cada UID, atribui os endereços `0x01` e
`0x02`, estabelece o enlace com SNRM/UA, negocia a versão e consulta os dados
RETAP. O processamento HDLC, enlace, procedimentos RETAP, configuração, alarmes
e tilt é feito diretamente pelo código C do firmware.

Antes de cada nova descoberta nesta porta virtual, o controlador envia
`Reset Device` XID em broadcast. Isso devolve os dois RETs a `NoAddress` mesmo
quando a aplicação anterior foi encerrada com o enlace ainda conectado. Esse
reset automático é restrito a `/tmp/aisg_controlador` e não é enviado em portas
seriais de hardware. A negociação de release também admite uma única repetição
se o primeiro ACK não for coletado pelo transporte.

## Formato do monitor

Cada quadro ocupa uma linha assim que é recebido. O formato segue o arquivo de
referência `dois rets.txt`: tempo desde a inicialização, sentido lógico,
octetos hexadecimais e intervalo desde o quadro anterior.

```text
[3803.65 ms] [TX] Quadro: 7E 01 93 8D B0 7E | Intervalo: 101.39 ms
[3808.41 ms] [RX] Quadro: 7E 01 73 83 57 7E | Intervalo: 4.76 ms
```

`TX` representa controlador → RET e `RX`, RET → controlador. Os octetos
usam dois algarismos hexadecimais em maiúsculas separados por um espaço. Com
`--detalhes`, o monitor acrescenta:

- `FCS=OK` ou `FCS=ERRO`, validado com CRC-16/X-25;
- endereço HDLC;
- tipo de controle I, S ou U, números de sequência e bit P/F;
- procedimento AISG 2.0 e comprimento declarado, quando houver informação.

No perfil **Serial RS-485 • AISG 2.0**, o controlador deixa pelo menos `1000 ms`
de barramento ocioso depois do último quadro recebido antes de transmitir o
próximo. O RET continua respondendo dentro de sua janela normal; por isso o
intervalo RX após TX permanece em torno de 4–9 ms e o próximo TX aparece cerca
de 1000 ms depois do RX.

Procedimentos assíncronos, como calibração, podem responder inicialmente com
`RR`. Isso confirma o I-frame, mas ainda não é a resposta RETAP. Depois da pausa,
o controlador envia um RR com `P=1` e `N(R)` preservado para buscar o resultado.
Se o trabalho ainda não terminou, o RET responde novamente com RR e o ciclo se
repete a cada segundo. Esse fluxo evita avançar indevidamente a sequência e
receber `FRMR`.

Na descoberta, o controlador lê os campos `MaximumTilt` (`0x06`) e
`MinimumTilt` (`0x07`) do próprio RET. O RET de bancada anuncia `0,0°..15,0°`.
O procedimento sRET `Set Tilt` (`0x33`) declara dois octetos de dados para o
alvo em décimos de grau. Seu tempo de bancada é
`|alvo - posição atual| × 2 s/°`; durante o deslocamento, operações de motor
usam o fluxo assíncrono RR/RR-P até a resposta RETAP final.

O exemplo de formatação `7F 01 0E F4 C3 C3 7F` também será impresso exatamente
assim se esses forem os bytes recebidos. Entretanto, em um quadro HDLC AISG
válido o delimitador é `7E`, não `7F`. O monitor não altera bytes inválidos para
fazê-los parecer válidos.

Por padrão, o mesmo conteúdo é acrescentado a
`simulador_serial/frames_aisg2.log`. Esse arquivo é ignorado pelo Git.

## Opções

```bash
python3 -m simulador_serial.simulador_serial --help
```

Opções principais:

- `--porta-controlador CAMINHO`: muda o link do controlador;
- `--porta-ret CAMINHO`: muda o link do RET;
- `--sem-links`: mostra e usa diretamente os dois `/dev/pts/N`;
- `--log ARQUIVO`: escolhe outro arquivo de log;
- `--sem-arquivo-log`: registra somente no terminal;
- `--detalhes`: acrescenta CRC, endereço e decodificação do controle;
- `--sem-ret-automatico`: deixa a segunda porta livre para outro processo;
- `--quantidade-rets N`: escolhe de 1 a 8 RETs automáticos (padrão: 2);
- `--endereco-ret N`: opcionalmente pré-atribui endereços sequenciais; sem
  esta opção, os RETs iniciam em `NoAddress` para exercitar a descoberta XID;
- `--executavel-ret CAMINHO`: usa um `ret_host` previamente compilado.

Para apenas validar um quadro copiado de outro log:

```bash
python3 -m simulador_serial.monitor_serial 7E 01 93 8D B0 7E
```

O comando retorna código `0` para quadro e FCS válidos e `1` para quadro
inválido. Também aceita uma linha por vez pela entrada padrão.

## Segurança da ponte

- dados parciais são mantidos em filas até o kernel aceitar todos os bytes;
- quadros fragmentados ou vários quadros na mesma leitura são reconstruídos;
- ruído anterior ao primeiro `7E` é encaminhado, mas não registrado como
  quadro;
- somente links simbólicos existentes podem ser substituídos. A ferramenta se
  recusa a apagar um arquivo comum que esteja no caminho solicitado;
- os links criados são removidos no encerramento normal por `Ctrl+C` ou SIGTERM.

Esta ferramenta observa a camada serial lógica. Ela não simula tensão RS-485,
controle de direção do transceptor, polarização do barramento ou alimentação
AISG de 24 V.

## Testes

```bash
python3 -m unittest discover -s simulador_serial/tests -v
```

Os testes cobrem formatação hexadecimal e temporal, CRC, transparência `7D`,
remontagem de quadros fragmentados, detecção de FCS inválido, tráfego
bidirecional entre as PTYs e uma sessão real `SNRM/UA + INITIAL_DATA` com o
`ret_core` C.
