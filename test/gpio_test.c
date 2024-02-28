typedef struct gpio {
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
}GPIO;

#define GPIOA (GPIO * 0x40020000)

enum {
    GPIO_MODE_INPUT,
    GPIO_MODE_OUTPUT,
    GPIO_MODE_AF,
    GPIO_MODE_ANALOG
    };

static set_pin_mode(GPIO *gpio,uint8_t pin_num,uint8_t mode){
    gpio->MODER &= ~(3<<(pin_num*2));
    gpio->MODER |= ((3*mode)<<(pin_num*2));
}

