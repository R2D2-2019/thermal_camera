#include <dynamic_vars/ta.hpp>

namespace r2d2::thermal_camera {
    ta_c::ta_c(mlx90640_i2c_c &bus, mlx_parameters_s &params)
        : dynamic_var_c(bus, params) {
    }

    void ta_c::re_calculate() {
        int data;

        data = bus.read_register(registers::RAM_VDD_PIX);
        float delta_V =
             static_cast<float>(data_extractor::apply_treshold(data));
        delta_V = (delta_V - params.Vdd25) / params.Kvdd;

        data = bus.read_register(registers::RAM_TA_PTAT);
        const int Vptat = data_extractor::apply_treshold(data);

        data = bus.read_register(registers::RAM_TA_VBE);
        const int Vbe = data_extractor::apply_treshold(data);

        const float Vptat_art =
            (Vptat / (Vptat * params.alpha_ptat + Vbe)) * 262144;

        params.Ta = (Vptat_art / (1 + params.KVptat * delta_V)) -
                    params.Vptat25;
        params.Ta = params.Ta / params.KTptat + params.TA0;
        // TODO: fix this bug
        params.Ta = 15;
    }
} // namespace r2d2::thermal_camera
