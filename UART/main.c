#include <stdbool.h>
#include <inttypes.h>

//Macros
#define BIT(x) (1U<<(x))
#define PINBANK(bank) ((bank)-'A')
#define FREQ 82000000
//enum for gpio modes
enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };
//structure to define RCC registers
typedef struct rcc {
  volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR,
      RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, AHB3ENR,
      RESERVED2, APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR,
      AHB3LPENR, RESERVED4, APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR,
      RESERVED6[2], SSCGR, PLLI2SCFGR;
}rcc;
#define RCC ((rcc *) 0x40023800)

//structure to define  GPIO registers
typedef struct gpio {
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
}gpio;
#define GPIO(bank) ((struct gpio *) (0x40020000 + 0x400 * (bank)))
//functiont to set gpio pins
static void set_pin_mode(uint8_t pin_bank,uint8_t pin_num,uint8_t mode){
    uint8_t bank = (pin_bank - 'A');
    gpio *gpio = GPIO(bank);
    RCC->AHB1ENR |=BIT(bank);
    gpio->MODER &= ~(3<<(pin_num*2));
    gpio->MODER |= ((3*mode)<<(pin_num*2));
}

//structure to define UART registers
typedef struct uart{
	volatile uint32_t SR,DR,BRR ,CR1,CR2,CR3,GTPR;
}uart;
#define UART1 ((struct uart *) 0x40011000)
#define UART2 ((struct uart *) 0x40004400)
#define UART3 ((struct uart *) 0x40004800)

//function to set alternate function
static inline gpio_set_af(uint8_t pin_bank,uint8_t pin_num, uint8_t af_num){
	struct gpio *gpio = GPIO(Pinbank(pin_bank));
	gpio->AFR[pin_num >> 3] &= ~(15UL << ((pin_num & 7) * 4));
	gpio->AFR[pin_num >> 3] |= ((uint32_t) af_num) << ((pin_num & 7) * 4);
}

//funtion to inintalize uart
static inline void uart_init(uart *uart , unsigned long baud ){
	uint8_t af =7;
	uint8_t tx_bank=0,rx_bank=0,tx_pin=0,rx_pin=0;
	if (uart == UART1)RCC->APB2ENR |= BIT(4);
	if (uart == UART2)RCC->APB1ENR |= BIT(17);
	if (uart == UART3)RCC->APB1ENR  |= BIT(5);

  if (uart == UART1){tx_bank = 'A',tx_pin=9,rx_bank='A',rx_pin=10;}
	if (uart == UART2){tx_bank = 'D',tx_pin=5,rx_bank='D',rx_pin=6;}
	if (uart == UART3){tx_bank = 'C',tx_pin=6,rx_bank='C',rx_pin=7;}
  set_pin_mode(tx_bank,tx_pin,GPIO_MODE_AF);
  set_pin_mode(rx_bank,rx_pin,GPIO_MODE_AF);
  gpio_set_af(tx_bank,tx_pin,af);
  gpio_set_af(rx_bank,rx_pin,af);
  uart->BRR |= FREQ/baud;
  uart->CR1 |= 0;
  uart->CR1 |= BIT(13)|BIT(2)|BIT(3); 
}

//function to read from uart
static inline uint8_t uart_read_byte(uart *uart){
  return (uint8_t) (uart->DR & 255);
}

//functio to write into uart reg
static inline void uart_write_byte(uart *uart,uint8_t byte){
  uart->DR |= byte;
  while ((uart->SR & BIT(7)) == 0) spin(1);
}
//write buffer for uart
static inline void uart_write_buffer(uart * uart ,char *buf,size_t len){
  while(len--) uart_write_byte(uart, *(uint8_t *) buf++);
}

//main code 

int main(){
  uart_init(UART3,115200);
  uart_write_buffer(UART3,"hi\r\n",4);

}