#include <cstddef>
#include <array>


class thermal_image_shell_c {
private:
	// For fast index checking
    const uint32_t image_height;
    const uint32_t image_width;

	// Array Pointer
	int16_t *pixel_container;

	// Find the pixel_container index from x, y coordinate
	uint32_t array_hash_index(uint32_t x, uint32_t y);
	
public: 
	// Constructor
	thermal_image_shell_c(const uint32_t image_height = 24, const uint32_t image_width = 32);

	// Destructor to deallocate pixel_container memory
	~thermal_image_shell_c();

	// Returns image height private member
	uint32_t get_image_height();

	// Returns image width private member
	uint32_t get_image_width();

	// Returns the data contained in x,y pixel. (at array index)
    int16_t get_pixel(const uint32_t x, const uint32_t y);

	// Sets the data in x,y pixel (at array index)
    void set_pixel(const uint32_t x, const uint32_t y, const int16_t pixel_data);
};