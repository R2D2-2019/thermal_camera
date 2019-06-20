#include <dynamic_vars/dynamic_var.hpp>

namespace r2d2::thermal_camera {
    class ta_c : public dynamic_var_c {
    public:
        ta_c(mlx90640_i2c_c &bus, mlx_parameters_s& params);

        void re_calculate() override;
    };
    
} // namespace r2d2::thermal_camera
