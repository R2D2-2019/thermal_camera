#include <iostream>
#include <thermal_image_shell.hpp>

int main( void ){

	auto thermal_image = thermal_image_shell_c();

	int counter = 0;
	for (unsigned int x = 0; x < thermal_image.get_image_width(); x++) {
        for (unsigned int y = 0; y < thermal_image.get_image_height(); y++) {
            thermal_image.set_pixel(x, y, counter);
            counter++;
		}
	}

	for (unsigned int x = 0; x < thermal_image.get_image_width(); x++) {
            for (unsigned int y = 0; y < thermal_image.get_image_height(); y++) {
                std::cout << "Pixel: " << y << "," << x << " = " << thermal_image.get_pixel(x, y) << "\n";
            }
        }

	std::cout << "Last Pixel: " << thermal_image.get_pixel(24, 32) << "\n";

	std::cout << "End of script\n";
    return 0;
}