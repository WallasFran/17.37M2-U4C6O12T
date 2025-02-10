# Projeto BitDogLab

## Descrição do Projeto

Neste projeto, você deverá utilizar os seguintes componentes conectados à placa BitDogLab:

- **Matriz 5x5 de LEDs (endereçáveis) WS2812**, conectada à GPIO 7.
- **LED RGB**, com os pinos conectados às GPIOs (11, 12 e 13).
- **Botão A** conectado à GPIO 5.
- **Botão B** conectado à GPIO 6.
- **Display SSD1306** conectado via I2C (GPIO 14 e GPIO 15).

## Funcionalidades do Projeto

### 1. Modificação da Biblioteca font.h
- Adicionar caracteres minúsculos à biblioteca `font.h`. Use sua criatividade para criar os novos caracteres.

### 2. Entrada de Caracteres via PC
- Utilize o **Serial Monitor** do VS Code para digitar os caracteres.
- Cada caractere digitado no Serial Monitor deve ser exibido no display SSD1306.
- **Nota:** Apenas um caractere será enviado de cada vez, não é necessário suportar o envio de strings completas.
- Quando um número entre 0 e 9 for digitado, um símbolo correspondente ao número deve ser exibido também na matriz 5x5 WS2812.

### 3. Interação com o Botão A
- **Pressionar o botão A** deve alternar o estado do LED RGB **Verde** (ligado/desligado).
- A operação deve ser registrada de duas formas:
  - Uma mensagem informativa sobre o estado do LED deve ser exibida no display SSD1306.
  - Um texto descritivo sobre a operação deve ser enviado ao **Serial Monitor**.

### 4. Interação com o Botão B
- **Pressionar o botão B** deve alternar o estado do LED RGB **Azul** (ligado/desligado).
- A operação deve ser registrada de duas formas:
  - Uma mensagem informativa sobre o estado do LED deve ser exibida no display SSD1306.
  - Um texto descritivo sobre a operação deve ser enviado ao **Serial Monitor**.

## Requisitos do Projeto

Para o desenvolvimento, devem ser seguidos os seguintes requisitos:

1. **Uso de interrupções:** Todas as funcionalidades relacionadas aos botões devem ser implementadas utilizando rotinas de interrupção (IRQ).
2. **Debouncing:** É obrigatório implementar o tratamento do bouncing dos botões via software.
3. **Controle de LEDs:** O projeto deve incluir o uso de LEDs comuns e LEDs WS2812, demonstrando o domínio de diferentes tipos de controle.
4. **Utilização do Display 128x64:** A utilização de fontes maiúsculas e minúsculas demonstrará o domínio do uso de bibliotecas, o entendimento do princípio de funcionamento do display, bem como, a utilização do protocolo I2C.
5. **Envio de informação pela UART:** Visa observar a compreensão sobre a comunicação serial via UART.
6. **Organização do código:** O código deve estar bem estruturado e comentado para facilitar o entendimento.

## Demonstração em Vídeo

Confira a demonstração do funcionamento do projeto no vídeo abaixo:

**[Demonstração do Projeto]**

(https://drive.google.com/file/d/1ohXLM7deo1BN2ty9yhXbfannFt8nPabA/view?usp=drive_link)

## Como Executar o Projeto

### Requisitos

- **Placa BitDogLab** com os componentes conectados corretamente:
  - Matriz 5x5 de LEDs WS2812 na GPIO 7
  - LED RGB na GPIOs 11, 12 e 13
  - Botões A e B na GPIOs 5 e 6
  - Display SSD1306 via I2C nas GPIOs 14 e 15
- **VS Code** com a extensão **Pico SDK** configurada
- **Biblioteca SSD1306** configurada

