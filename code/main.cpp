#include "hwlib.hpp"
#include "MLX90640_I2C_Driver.h"
#include "MLX90640_API.h"
int main(void) {
  // kill the watchdog
    WDT->WDT_MR = WDT_MR_WDDIS;
    hwlib::wait_ms(1000);
    for (;;){
        hwlib::cout << "this works via arduino";
        hwlib::wait_ms(1000);
    }
}