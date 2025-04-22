// Inclusão de Bibliotecas
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "revisao.pio.h"

#define I2C_PORT i2c1 // Porta I2C
#define I2C_SDA 14 // GPIO para SDA do LCD
#define I2C_SCL 15 // GPIO para SCL do LCD
#define endereco 0x3C // Endereço do display
#define LED_PIN_RED 13 // LED vermelho
#define JOYSTICK_X_PIN 27  // GPIO para eixo X
#define JOYSTICK_Y_PIN 26  // GPIO para eixo Y
#define BOTAO_A 5 // GPIO para botão A
#define BOTAO_B 6 // GPIO para botão B
#define LED_PIN 7 // GPIO para Matriz de LEDs
#define BUZZER_A 21 // GPIO para o buzzer A
#define BUZZER_B 10 // GPIO para o buzzer B

ssd1306_t ssd; // Inicializa a estrutura do display

volatile uint cont = 0; // Contador de interrupções

// Váriavel para debounce
volatile absolute_time_t last_interrupt_time_button_a = 0;

#include "pico/bootrom.h" // Inclusão da biblioteca para interrupção do Botão B

// Interrupção para os botões
void gpio_irq_handler(uint gpio, uint32_t events) {
if (gpio == BOTAO_A) // Botão A
{
  absolute_time_t time = get_absolute_time();
  if (absolute_time_diff_us(last_interrupt_time_button_a, time) > 200000) // Debounce de 200ms
  {
    cont++; // Incrementa o contador
    if (cont > 1) 
    {
      cont = 0; // Reseta o contador
    }
  }
  last_interrupt_time_button_a = time;
} else if (gpio == BOTAO_B) { // Botão B
   reset_usb_boot(0, 0); // Reseta o Pico e entra no modo de inicialização USB
  }
}

// Matriz de LEDs

#define NUM_PIXELS 25 // Número de LEDs na matriz
uint matrix_rgb(float r, float g, float b) // Função para converter RGB em um valor de 32 bits
{
  unsigned char R, G, B;
  R = r * 255;
  G = g * 255;
  B = b * 255;
  return (G << 24) | (R << 16) | (B << 8);
}

// Função para converter a posição do matriz para uma posição do vetor.
int getIndex(int x, int y)
{
  // Se a linha for par (0, 2, 4), percorremos da esquerda para a direita.
  // Se a linha for ímpar (1, 3), percorremos da direita para a esquerda.
  if (y % 2 == 0)
  {
    return 24 - (y * 5 + x); // Linha par (esquerda para direita).
  }
  else
  {
    return 24 - (y * 5 + (4 - x)); // Linha ímpar (direita para esquerda).
  }
}

// Função para desenhar na matriz

void desenho_pio(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b)
{

  for (int16_t i = 0; i < NUM_PIXELS; i++)
  {
    valor_led = matrix_rgb(desenho[i] * r, desenho[i] * g, desenho[i] * b);
    pio_sm_put_blocking(pio, sm, valor_led);
  };
}

// Sprites
double apagar_leds[25] =      // Apagar LEDs da matriz
 {0.0, 0.0, 0.0, 0.0, 0.0,    
  0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0};

double icone_mais[25] =       // + (Positivo em relação ao ponto central)
 {0.0, 0.0, 1.0, 0.0, 0.0,    
  0.0, 0.0, 1.0, 0.0, 0.0,
  1.0, 1.0, 1.0, 1.0, 1.0,
  0.0, 0.0, 1.0, 0.0, 0.0,
  0.0, 0.0, 1.0, 0.0, 0.0};

double icone_menos[25] =      // - (Negativo em relação ao ponto central)
 {0.0, 0.0, 0.0, 0.0, 0.0,    
  0.0, 0.0, 0.0, 0.0, 0.0,
  1.0, 1.0, 1.0, 1.0, 1.0,
  0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0};

// Configuração dos pinos
void setup() {
  // Configuração dos botões
  gpio_init(BOTAO_A);
  gpio_set_dir(BOTAO_A, GPIO_IN);
  gpio_pull_up(BOTAO_A);

  gpio_init(BOTAO_B);
  gpio_set_dir(BOTAO_B, GPIO_IN);
  gpio_pull_up(BOTAO_B);

  // Configuração I2C/display
  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
  gpio_pull_up(I2C_SDA); // Pull up the data line
  gpio_pull_up(I2C_SCL); // Pull up the clock line

  // Inicialização do ADC para o joystick
  adc_init();
  adc_gpio_init(JOYSTICK_X_PIN); // Canal 1 (GPIO27)
  adc_gpio_init(JOYSTICK_Y_PIN); // Canal 0 (GPIO26)

  // Configuração da Matriz de LEDs
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
}

// Configura PWM para um GPIO específico
void pwm_setup(uint8_t GPIO) {
  gpio_set_function(GPIO, GPIO_FUNC_PWM); // Define função PWM para o pino
  uint slice_num = pwm_gpio_to_slice_num(GPIO); // Obtém o número do slice
  pwm_config config = pwm_get_default_config(); // Configuração padrão
  pwm_config_set_wrap(&config, 4095); // Wrap em 4095 para 12 bits
  pwm_init(slice_num, &config, true); // Inicializa PWM
}

// Função Principal
int main() {
  stdio_init_all(); // Inicializa comunicação serial
  setup(); // Configuração inicial
  
  gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
  gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

// I2C Initialisation. Using it at 400Khz.
  i2c_init(I2C_PORT, 400 * 1000);
  
  ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
  ssd1306_config(&ssd); // Configura o display
  ssd1306_send_data(&ssd); // Envia os dados para o display

  // Limpa o display. O display inicia com todos os pixels apagados.
  ssd1306_fill(&ssd, false); // Limpa o display
  ssd1306_send_data(&ssd);

  // Configura PWM para Buzzer A e B
  pwm_setup(BUZZER_A);
  pwm_setup(BUZZER_B);
  pwm_setup(LED_PIN_RED); // Configura PWM para LED vermelho

  // Configurações da PIO/Matriz de LEDs
  PIO pio = pio0;
  bool frequenciaClock;
  uint16_t i;
  uint valor_led;
  float r = 0.0, b = 0.0, g = 0.0;

  frequenciaClock = set_sys_clock_khz(128000, false); // frequência de clock de 128MHz

  uint offset = pio_add_program(pio, &pio_matrix_program);
  uint sm = pio_claim_unused_sm(pio, true);
  pio_matrix_program_init(pio, sm, offset, LED_PIN);

  // Declaração de variáveis do loop
  bool cor = true;
  static bool led_red_state = false;
  uint16_t adc_value_x, adc_value_y;
  
  // Variáveis de controle de tempo
  static absolute_time_t time_sleep = 0;
  static absolute_time_t last_display_update = 0;
  static absolute_time_t last_led_update = 0;
  static absolute_time_t last_uart_log = 0;
  static absolute_time_t last_toggle_red = 0;


  // Loop infinito
  while (true) {
    // Lê valores do joystick
    adc_select_input(0); // Canal Y (ADC0)
    adc_value_y = adc_read();
    adc_select_input(1); // Canal X (ADC1)
    adc_value_x = adc_read();
    absolute_time_t current_time = get_absolute_time();
    
    
    // Matriz LEDs
    if (absolute_time_diff_us(last_led_update, current_time) > 50000) { // 50ms
      switch (cont) {
        {
        case 0:
          if (adc_value_x > 2100) {
            desenho_pio(icone_mais, valor_led, pio, sm, 0.0, 1.0, 0.0); // Desenha o ícone + (positivo)
          } else if (adc_value_x < 1900) {
            desenho_pio(icone_menos, valor_led, pio, sm, 1.0, 0.0, 0.0); // Desenha o ícone - (negativo)
          } else {
            desenho_pio(apagar_leds, valor_led, pio, sm, 0.0, 0.0, 0.0); // Apaga LEDs
          }
          break;
          case 1:
          if (adc_value_y > 2100) {
            desenho_pio(icone_mais, valor_led, pio, sm, 0.0, 1.0, 0.0); // Desenha o ícone + (positivo)
          } else if (adc_value_y < 1900) {
            desenho_pio(icone_menos, valor_led, pio, sm, 1.0, 0.0, 0.0); // Desenha o ícone - (negativo)
          } else {
            desenho_pio(apagar_leds, valor_led, pio, sm, 0.0, 0.0, 0.0); // Apaga LEDs
          }
          break;
        default:
          break;
        }
      last_led_update = current_time;
    }
  }

    // OLED

    int square_x , square_y;
 
     // Mapeamento linear para X
    square_x = (adc_value_x * 120) / 4095;
     
     // Mapeamento linear para Y
    square_y = 56 - (adc_value_y * 56) / 4095;
     
     // Limites
    if (square_x < 8) square_x = 0;
    if (square_x > 120) square_x = 120;
    if (square_y < 8) square_y = 4;
    if (square_y > 56) square_y = 56;
    
    if (absolute_time_diff_us(last_display_update, current_time) > 10000) {
      ssd1306_fill(&ssd, false); // Limpa o display      
      ssd1306_rect(&ssd, square_y, square_x, 8, 8, true, true); // Desenha o quadrado
      ssd1306_send_data(&ssd); // Atualiza o display
      last_display_update = current_time;
    }

    // UART
    if (absolute_time_diff_us(last_uart_log, current_time) > 500000) { // 500ms de intervalo entre logs
      printf("Posição no display: \nX: %d, Y: %d \n", square_x, square_y); // Imprime valores do joystick
      printf("Pressione o botão A para alterar o eixo de referência (X ou Y)\n"); // Imprime valores do joystick
      last_uart_log = current_time;
    }

    // Joystick X

    uint difference;

    if (cont == 0){
      // Calcula diferença e direção
      if (adc_value_x > 2100) {
        difference = adc_value_x - 2048;
      } else if (adc_value_x < 1900) {
          difference = 2048 - adc_value_x;
      } else {
          pwm_set_gpio_level(LED_PIN_RED, 0);
          led_red_state = false; // Resetar estado
          continue; // Saída do loop
        }
    } else if (cont == 1){
        if (adc_value_y > 2100) {
          difference = adc_value_y - 2048;
      } else if (adc_value_y < 1900) {
          difference = 2048 - adc_value_y;
      } else {
          pwm_set_gpio_level(LED_PIN_RED, 0);
          led_red_state = false; // Resetar estado
          continue; // Saída do loop
        }
    }

    // Calcula intervalo de piscada (inversamente proporcional à diferença)
    uint32_t max_sleep = 1000000;   // 1 segundo para diferença mínima
    uint32_t min_sleep = 100000;     // 0.1 segundos para diferença máxima
    time_sleep = max_sleep - ((difference * (max_sleep - min_sleep)) / 2048);

    // Verifica se é hora de alternar o estado
    if (absolute_time_diff_us(last_toggle_red, current_time) >= time_sleep) {
        led_red_state = !led_red_state;
        
        if (led_red_state) { // Liga o LED e o buzzer
            pwm_set_gpio_level(LED_PIN_RED, difference * 2);
            pwm_set_gpio_level(BUZZER_A, difference * 2);
            pwm_set_gpio_level(BUZZER_B, difference * 2);
        } else { // Desliga o LED e o buzzer
            pwm_set_gpio_level(LED_PIN_RED, 0);
            pwm_set_gpio_level(BUZZER_A, 0);
            pwm_set_gpio_level(BUZZER_B, 0);
        }
        
        last_toggle_red = current_time; // Atualiza o tempo da última alternância
    }
    sleep_ms(100); // Atraso de 100ms para evitar sobrecarga
  }
  return 0;
}
