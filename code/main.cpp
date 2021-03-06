#include <limits>
#include <math.h>
#include <mlx90640.hpp>

using namespace r2d2::thermal_camera;
using namespace r2d2::i2c;

int main() {
    // kill the watchdog
    WDT->WDT_MR = WDT_MR_WDDIS;
    hwlib::wait_ms(1000);
    i2c_bus_c bus(i2c_bus_c::interface::interface_1, 400'000);
    mlx90640_c thermal_cam(bus);

    while (true) {
        thermal_cam.set_frame();
        for (const auto &row : *thermal_cam.get_frame_ptr()) {
            for (const auto pixel : row) {
                hwlib::cout << static_cast<int>(pixel) << hwlib::setw(5) << ' ';
            }
            hwlib::cout << hwlib::endl;
        }
        hwlib::cout << hwlib::endl << hwlib::endl << hwlib::endl;
    }
}