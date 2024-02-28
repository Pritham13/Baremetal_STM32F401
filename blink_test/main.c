//TODO: check pinbank logic n line 33 and similar ones

#include <stdbool.h>
#include <inttypes.h>

//Macros
#define BIT(x) (1U<<(x))
#define PIN (bank,num) (((bank)-'A')<<8|num)
#define PINBANK(pin) (pin>>8)
#define PINNO(pin) (pin & 255)
#define GPIO(bank) ((struct gpio *) (0x40020000 + 0x400 * (bank))
#define RCC ((struct rcc *) 0x40023800)

//Structures
typedef struct gpio {
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
}gpio;
typedef struct rcc {
  volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR,
      RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, AHB3ENR,
      RESERVED2, APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR,
      AHB3LPENR, RESERVED4, APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR,
      RESERVED6[2], SSCGR, PLLI2SCFGR;
}rcc;


//enums 
// Enum values are per datasheet: 0, 1, 2, 3
enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };

// gpio set  with clock enbaling to set the pin as output or input
static set_pin_mode(uint8_t pin_bank,uint8_t pin_num,uint8_t mode){
    uint8_t bank = (pin_bank - 'A');
    gpio *gpio = GPIO(bank);
    RCC->AHB1ENR |=BIT(bank);
    gpio->MODER &= ~(3<<(pin_num*2));
    gpio->MODER |= ((3*mode)<<(pin_num*2));
}

static gpio_write(uint8_t pin_bank,uint8_t pin_num,bool val){
  uint8_t bank = (pin_bank - 'A');
  gpio *gpio =GPIO(bank);
  gpio->BSRR = (1U << pin_num << (val ? 0 : 16));
}

static inline  void spin(volatile uint32_t count){
  while (count--) (void) 0 ;
}


int main(){
  set_pin_mode('B',7,GPIO_MODE_OUTPUT);
  for(;;) {
      gpio_write('B',7, true);
      spin(999999);
      gpio_write('B',7, false);
      spin(999999);
  }

  return 0;
}

// Startup code
__attribute__((naked, noreturn)) void _reset(void) {
  // memset .bss to zero, and copy .data section to RAM region
  extern long _sbss, _ebss, _sdata, _edata, _sidata;
  for (long *dst = &_sbss; dst < &_ebss; dst++) *dst = 0;
  for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;) *dst++ = *src++;

  main();             // Call main()
  for (;;) (void) 0;  // Infinite loop in the case if main() returns
}

extern void _estack(void);  // Defined in link.ld

// 16 standard and 91 STM32-specific handlers
__attribute__((section(".vectors"))) void (*const tab[16 + 91])(void) = {
    _estack, _reset};