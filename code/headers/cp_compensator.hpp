#include <mlx_extractor.hpp>

namespace r2d2::thermal_camera {
    /**
     * Gets the compensation of the gain of the cp (corner pixel)
     */
    class cp_compensator_c : public mlx_extractor_c {
    private:
        reading_pattern pattern;

    public:
        cp_compensator_c(mlx90640_i2c_c &bus, const reading_pattern pattern);

        void extract(mlx_parameters_s &params) override;
    };
} // namespace r2d2::thermal_camera
