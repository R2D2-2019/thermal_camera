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
	
	std::cout << "Image height: " << thermal_image.get_image_height() << "\n";
    std::cout << "Image width: " << thermal_image.get_image_width() << "\n";
	std::cout << "Pixel 1: " << thermal_image.get_pixel(0, 0) << "\n";
    std::cout << "Pixel 2: " << thermal_image.get_pixel(1, 1) << "\n";
    std::cout << "Pixel 3: " << thermal_image.get_pixel(31, 23) << "\n";
	std::cout << "Pixel 4: " << thermal_image.get_pixel(32, 24) << "\n";


	std::cout << "End of code\n";
    return 0;
}