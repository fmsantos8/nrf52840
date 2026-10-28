# nRF52840 HAL port

Implementa os contratos públicos do framework em C11 com nRF Connect SDK
v3.1.0, Zephyr e os drivers nrfx. A aplicação fornece a arena a
`hal_system_init()` e fecha a construção com `hal_system_seal()`.
Objetos concretos ficam privados neste repositório e são alocados na arena.

O port recebe pinos e configurações do BSP. O overlay seleciona as rotas
de I2C e o endpoint serial pelos aliases `hal-i2c0` e `hal-uart0`.
O port não contém os pinos nem a lógica da prova de conceito.

| Recurso | Suporte |
| --- | --- |
| GPIO | P0.00–P0.31 e P1.00–P1.15 disponíveis no GPIO do SDK; níveis físicos; entrada ou saída push-pull |
| I2C, instância 0 | TWIM0; rota igual ao pinctrl do alias; 100/250/400 kHz; endereço 0x08–0x77; 1–256 bytes por transação com STOP |
| Serial, instância 0 | USB CDC ACM do alias; perfil 115200 8N1; TX/RX devem usar `HAL_PIN_NONE`; nenhum GPIO é reservado |
| Timer, instância 2 | TIMER2 dedicado, 32 bits, 1 MHz; períodos inteiros de 1 a 4.294.967 ms; callback em interrupção |

Outras instâncias/configurações válidas retornam `HAL_STATUS_NOT_SUPPORTED`.
Identificadores e argumentos inválidos retornam `HAL_STATUS_INVALID_ARGUMENT`.
O Zephyr deve inicializar kernel, clocks e GPIO antes de chamar o port.
TWIM0 e TIMER2 ficam desabilitados no devicetree para impedir que drivers
Zephyr concorram com os drivers nrfx usados pelo framework.

## Esperas e propriedade dos dados

I2C e serial aceitam timeout de 1 a `INT32_MAX` ms. As esperas usam os ticks
do kernel, arredondados pela resolução do Zephyr, com possível atraso de
escalonamento; o valor não é um prazo estrito para a chamada inteira.

O I2C usa nrfx assíncrono e uma espera finita por conclusão. NACK retorna
`HAL_STATUS_ERROR`; expiração retorna `HAL_STATUS_TIMEOUT`. Ao expirar, o
port desabilita TWIM/EasyDMA, limpa o IRQ pendente e reabilita o controlador.
Não aguarda STOP indefinidamente nem repete a transação. Esse procedimento
libera o driver para a próxima chamada; não garante recuperação elétrica de
um dispositivo mantendo SDA/SCL presos. O buffer persistente de 256 bytes
permite escrever fontes em Flash e impede que o SDK retenha buffers do chamador.
Leituras só copiam o resultado para o destino depois do sucesso.

A USB inicia sem aguardar o computador. A transmissão aguarda DTR e espaço
na fila CDC dentro do timeout; desconexão ou falta de progresso retorna
`HAL_STATUS_TIMEOUT`, com a contagem do prefixo aceito. A fila do SDK recebe
cópias dos bytes. Sucesso confirma essa cópia; entrega ao host depende da USB
e não é confirmada. Dados aceitos antes de timeout podem sair depois do retorno.
O driver nunca mantém um ponteiro para o buffer fornecido pela aplicação.
O baud rate identifica o perfil serial; a USB não transmite a 115200 bits/s.

O callback do TIMER2 roda em interrupção e deve ser curto e não bloqueante.
O timer começa somente em `hal_timer_start()`, uma única vez, inclusive depois
de selar a arena. Latência pode atrasar ou aglutinar notificações; não há
compensação de períodos perdidos. O RTC do Zephyr permanece separado do TIMER2.

Construção, I2C e serial são sequenciais em contexto de thread. A integração
serializa o acesso aos recursos; não há suporte a transferências concorrentes.
O BSP também deve evitar pinos e instâncias já usados por outros subsistemas
do SDK. Não existe destruição, liberação nem reconfiguração dos objetos.
Falha depois de alocar pode consumir arena, conforme o contrato público.

## Integração e verificações

A integração compila os seis fontes do port, `hal_memory.c` e os módulos
Zephyr CMSIS/CMSIS 6/hal_nordic. Habilita `CONFIG_GPIO`, `CONFIG_PINCTRL`,
`CONFIG_NRFX_TWIM0`, `CONFIG_NRFX_TIMER2`, `CONFIG_SERIAL`,
`CONFIG_UART_INTERRUPT_DRIVEN`, `CONFIG_UART_LINE_CTRL`,
`CONFIG_USB_DEVICE_STACK` e `CONFIG_USB_CDC_ACM`.
Desabilita a inicialização automática da USB e os consoles que disputariam a serial.

Os testes de contrato do projeto de integração exercitam estes fontes com
fronteiras de SDK simuladas: ciclo de vida, arena, recursos, falhas, NACK,
timeout/recuperação, desconexão USB, envio parcial, callback e restrições em ISR.
Os builds ARM usam o SDK real e produzem ELF/HEX/BIN/UF2. Esses testes não
substituem a verificação das ligações, USB e periféricos na placa física.
