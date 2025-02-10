#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2818b.pio.h"

// Definindo pinos para o LED RGB e botões
#define LED_VERMELHO_PIN 13
#define LED_VERDE_PIN 11
#define LED_AZUL_PIN 12
#define BOTAO_A_PIN 5
#define BOTAO_B_PIN 6

// Definindo I2C
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define ENDERECO 0x3C

// Definindo o tempo de debounce em milissegundos
#define DEBOUNCE_TIME 400

// Definindo o número de LEDs e pino da matriz 5x5 WS2812
#define LED_COUNT 25
#define LED_PIN 7

// Definindo o número de pixels
#define NUM_PIXELS LED_COUNT

// Declaração do buffer de pixels da matriz 5x5
struct pixel_t {
    uint8_t G, R, B; // Três valores de 8-bits compõem um pixel.
};
typedef struct pixel_t npLED_t;
npLED_t leds[LED_COUNT];

// Variáveis para uso da máquina PIO.
PIO np_pio;
uint sm;

// Variáveis globais
ssd1306_t ssd;
char caractere_atual = '\0';  // Variável global para armazenar o caractere digitado
char letra_atual = '\0';  // Variável global para armazenar a letra digitada (A-Z)

// Variáveis para controle de debounce
uint32_t last_interrupt_time_a = 0;
uint32_t last_interrupt_time_b = 0;

// Função para configurar pinos
void setup_pinos() {
    // Configura os pinos dos LEDs como saída
    gpio_init(LED_VERMELHO_PIN);
    gpio_set_dir(LED_VERMELHO_PIN, GPIO_OUT);
    gpio_put(LED_VERMELHO_PIN, 0); // Inicialmente desligado

    gpio_init(LED_VERDE_PIN);
    gpio_set_dir(LED_VERDE_PIN, GPIO_OUT);
    gpio_put(LED_VERDE_PIN, 0); // Inicialmente desligado

    gpio_init(LED_AZUL_PIN);
    gpio_set_dir(LED_AZUL_PIN, GPIO_OUT);
    gpio_put(LED_AZUL_PIN, 0); // Inicialmente desligado
    
    // Configura os pinos dos botões como entrada com pull-up
    gpio_init(BOTAO_A_PIN);
    gpio_set_dir(BOTAO_A_PIN, GPIO_IN);
    gpio_pull_up(BOTAO_A_PIN);
    
    gpio_init(BOTAO_B_PIN);
    gpio_set_dir(BOTAO_B_PIN, GPIO_IN);
    gpio_pull_up(BOTAO_B_PIN);
}

// Função para configurar o display SSD1306
void setup_display(ssd1306_t *ssd) {
    // Inicializa o display SSD1306 com as configurações definidas
    ssd1306_init(ssd, WIDTH, HEIGHT, false, ENDERECO, I2C_PORT);
    ssd1306_config(ssd);
    ssd1306_send_data(ssd);
    ssd1306_fill(ssd, false);
    ssd1306_send_data(ssd);
}

// Função para alternar o estado do LED verde
void alternar_led_verde(bool estado) {
    gpio_put(LED_VERDE_PIN, estado);
}

// Função para alternar o estado do LED azul
void alternar_led_azul(bool estado) {
    gpio_put(LED_AZUL_PIN, estado);
}

// Função para exibir uma mensagem no display SSD1306
void exibir_mensagem(ssd1306_t *ssd, const char *mensagem, int x_pos, int y_pos) {
    ssd1306_fill(ssd, false);  // Limpa a tela
    ssd1306_draw_string(ssd, mensagem, x_pos, y_pos); // Desenha o texto na posição desejada
    ssd1306_send_data(ssd);   // Atualiza o display
}

// Função para inicializar a matriz WS2812
void npInit(uint pin) {
    // Adiciona o programa WS2818 à máquina PIO
    uint offset = pio_add_program(pio0, &ws2818b_program);
    np_pio = pio0;

    sm = pio_claim_unused_sm(np_pio, false);
    if (sm < 0) {
        np_pio = pio1;
        sm = pio_claim_unused_sm(np_pio, true); // Se nenhuma máquina estiver livre, panic!
    }

    // Inicializa o programa WS2818 na máquina PIO
    ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);

    // Inicializa os LEDs como desligados
    for (uint i = 0; i < LED_COUNT; ++i) {
        leds[i].R = 0;
        leds[i].G = 0;
        leds[i].B = 0;
    }
}

// Função para atribuir uma cor RGB a um LED
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) {
    leds[index].R = r;
    leds[index].G = g;
    leds[index].B = b;
}

// Função para limpar o buffer de pixels
void npClear() {
    for (uint i = 0; i < LED_COUNT; ++i)
        npSetLED(i, 0, 0, 0);
}

// Função para escrever os dados do buffer nos LEDs
void npWrite() {
    for (uint i = 0; i < LED_COUNT; ++i) {
        pio_sm_put_blocking(np_pio, sm, leds[i].G);
        pio_sm_put_blocking(np_pio, sm, leds[i].R);
        pio_sm_put_blocking(np_pio, sm, leds[i].B);
    }
    sleep_us(100); // Espera 100us, sinal de RESET do datasheet.
}

// Função para mapear as coordenadas X, Y para a posição do LED na matriz
int getIndex(int x, int y) {
    if (y % 2 == 0) {
        return 24 - (y * 5 + x); // Linha par (esquerda para direita).
    } else {
        return 24 - (y * 5 + (4 - x)); // Linha ímpar (direita para esquerda).
    }
}

// Função para desenhar um número na matriz 5x5
void desenharNumero(int numero) {
    bool numeros[10][NUM_PIXELS] = {
        {0, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0},  // 0
        {0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0},  // 1
        {0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0},  // 2
        {0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0},  // 3
        {0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0},  // 4
        {0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0},  // 5
        {0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0},  // 6
        {0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0},  // 7
        {0, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0},  // 8
        {0, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0}   // 9
    };

    npClear();

    // Desenha o número na matriz 5x5
    for (int linha = 0; linha < 5; linha++) {
        for (int coluna = 0; coluna < 5; coluna++) {
            int posicao = getIndex(coluna, linha);
            if (numeros[numero][linha * 5 + coluna]) {
                npSetLED(posicao, 40, 0, 0); // Cor vermelha
            }
        }
    }

    npWrite(); // Atualiza a matriz de LEDs
}

// Função para lidar com interrupções dos botões
void gpio_irq_handler(uint gpio, uint32_t events) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    char mensagem[20];

    // Tratamento do botão A
    if (gpio == BOTAO_A_PIN) {
        if (current_time - last_interrupt_time_a > DEBOUNCE_TIME) {
            last_interrupt_time_a = current_time;

            static bool led_verde_estado = false;
            led_verde_estado = !led_verde_estado;
            alternar_led_verde(led_verde_estado);

            // Exibe no display se o LED verde está ligado ou desligado
            if (led_verde_estado) {
                snprintf(mensagem, sizeof(mensagem), "Verde Ligado");
            } else {
                snprintf(mensagem, sizeof(mensagem), "Verde Desligado");
            }

            exibir_mensagem(&ssd, mensagem, 0, 30);
            printf("Botão A pressionado. %s\n", mensagem);
        }
    }

    // Tratamento do botão B
    if (gpio == BOTAO_B_PIN) {
        if (current_time - last_interrupt_time_b > DEBOUNCE_TIME) {
            last_interrupt_time_b = current_time;

            static bool led_azul_estado = false;
            led_azul_estado = !led_azul_estado;
            alternar_led_azul(led_azul_estado);

            // Exibe no display se o LED azul está ligado ou desligado
            if (led_azul_estado) {
                snprintf(mensagem, sizeof(mensagem), "Azul Ligado");
            } else {
                snprintf(mensagem, sizeof(mensagem), "Azul Desligado");
            }

            exibir_mensagem(&ssd, mensagem, 0, 30);
            printf("Botão B pressionado. %s\n", mensagem);
        }
    }
}

int main() {
    // Inicializando as configurações
    stdio_init_all();
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Configuração dos pinos, display e WS2812
    setup_pinos();
    setup_display(&ssd);
    npInit(LED_PIN);

    // Configura interrupções para os botões
    gpio_set_irq_enabled_with_callback(BOTAO_A_PIN, GPIO_IRQ_EDGE_FALL, true, gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BOTAO_B_PIN, GPIO_IRQ_EDGE_FALL, true, gpio_irq_handler);

    // Exibe mensagem inicial no display
    exibir_mensagem(&ssd, "Digite um char:", 0, 0);

    // Loop principal
    while (true) {
        // Lê o caractere do Serial Monitor
        if (scanf("%c", &caractere_atual) == 1) {
            // Exibe o caractere no display SSD1306
            char mensagem[20];
            snprintf(mensagem, sizeof(mensagem), "Caractere %c", caractere_atual);
            exibir_mensagem(&ssd, mensagem, 0, 10);

            // Exibe o caractere na matriz WS2812 se for um número
            if (caractere_atual >= '0' && caractere_atual <= '9') {
                desenharNumero(caractere_atual - '0');
            }

            // Aguarda para manter o caractere na tela por um tempo
            sleep_ms(400);  // Aguarda 400 ms (0.4 segundo)
        }
    }

    return 0;
}