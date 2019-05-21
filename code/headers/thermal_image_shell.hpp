#include <cstddef>
#include <array>

template <std::size_t image_height, std::size_t image_width>
class thermal_image_shell_c {
private:

	std::array<uint16_t, image_height*image_width> pixel_container;

public: 
	thermal_image_shell_c();
};