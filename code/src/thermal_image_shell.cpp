#include "thermal_image_shell.hpp"

uint32_t thermal_image_shell_c::array_hash_index(uint32_t x, uint32_t y) {

	if (y > image_height) {
        y = image_height;
    }

	if (x > image_width) {
        x = image_width;
	}

    uint32_t new_index = (y * image_width) + x;
	
	return new_index;
}

thermal_image_shell_c::thermal_image_shell_c(const uint32_t image_height, const uint32_t image_width): 
	image_height(image_height), 
	image_width(image_width)
	{
		pixel_container = new int16_t[image_height * image_width];
	}

thermal_image_shell_c::~thermal_image_shell_c() {

	delete[] pixel_container;
}

uint32_t thermal_image_shell_c::get_image_height() {

    return image_height;
}

uint32_t thermal_image_shell_c::get_image_width() {

    return image_width;
}

int16_t thermal_image_shell_c::get_pixel(const uint32_t x, const uint32_t y) {

    int16_t pixel_data = pixel_container[array_hash_index(x, y)];

	return pixel_data;
}

void thermal_image_shell_c::set_pixel(const uint32_t x, const uint32_t y, const int16_t pixel_data) {

	pixel_container[array_hash_index(x, y)] = pixel_data;
}