#pragma once

#include <pixel_manipulator.hpp>
#include <kta.hpp>
#include <kv.hpp>
#include <pix_os_ref.hpp>

namespace r2d2::thermal_camera {
    class pix_os_c : public pixel_manipulator_c {
    private:
        lookupable_c &kta;
        lookupable_c &kv;
        lookupable_c &pix_os_ref;

    public:
        pix_os_c(mlx_parameters_s &params,
                 std::array<std::array<float, 32>, 24> &pixels, lookupable_c &kta,
                 lookupable_c &kv, lookupable_c &pix_os_ref);

        void calculate_pixel(unsigned int row, unsigned int col) override;
    };
} // namespace r2d2::thermal_camera
