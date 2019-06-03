#include <mlx90640.hpp>

using namespace r2d2::thermal_camera;
using namespace r2d2::i2c;

int main() {
    // kill the watchdog
    WDT->WDT_MR = WDT_MR_WDDIS;
    hwlib::wait_ms(1000);
    i2c_bus_c bus(i2c_bus_c::interface::interface_1, 100'000);
    mlx90640_c thermal_cam(bus);
}