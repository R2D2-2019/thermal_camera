#include <alpha_comp_extractor.hpp>
#include <cmath>
#include <cp_compensator.hpp>
#include <data_extractor.hpp>
#include <kgain.hpp>
#include <kv_coefficient.hpp>
#include <mlx90640_processor.hpp>
#include <patron_extractor.hpp>
#include <pix_os.hpp>
#include <ta_extractor.hpp>
#include <tgc_extractor.hpp>
#include <to_extractor.hpp>
#include <vir_extractor.hpp>

namespace r2d2::thermal_camera {

    mlx90640_processor_c::mlx90640_processor_c(mlx90640_i2c_c &bus,
                                               float emissivity)
        : bus(bus) {
        set_resolution_correlation(); // datasheet page 35, just 1 call
        set_Vdd();                    // datasheet page 36, just 1 call
        params.emissivity = emissivity;
    }
    // Function is only to be called once.
    void mlx90640_processor_c::set_resolution_correlation() {
        uint16_t data =
            bus.read_register(registers::EE_CTRL_CALIB_KV_KTA_SCALE);
        uint16_t resolution_ee =
            data_extractor_s::extract_data(data, 0x3000, 12);
        data = bus.read_register(registers::INTERNAL_CONTROL_REGISTER);
        uint16_t resolution_reg =
            data_extractor_s::extract_data(data, 0x0C00, 10);
        // (2^resolution_ee) = (1 << resolution_ee)
        resolution_ee = (1u << resolution_ee);
        resolution_reg = (1u << resolution_reg);
        params.res_cor = static_cast<uint8_t>(resolution_ee / resolution_reg);
    }

    // Function is only to be called once.
    void mlx90640_processor_c::set_Vdd() {
        int data = bus.read_register(registers::EE_VDD_PIX);

        params.Kvdd =
            data_extractor_s::extract_and_treshold(data, 0xFF00, 8, 127, 256);
        // (Kvdd * 2^y) = (Kvdd << y)
        params.Kvdd <<= 5;

        params.Vdd25 = data_extractor_s::extract_data(data, 0x00FF, 0);
        // (x * 2^y) = (x << y)
        params.Vdd25 = ((params.Vdd25 - 256) << 5) - 8192;

        data = bus.read_register(registers::RAM_VDD_PIX);
        const int ram_vdd_pix = data_extractor_s::apply_treshold(data);
        params.Vdd =
            ((params.res_cor * ram_vdd_pix - params.Vdd25) / params.Kvdd) +
            params.VDD0;
    }

    // Pipeline
    float mlx90640_processor_c::get_temperature_pixel(int row, int col) {
        data_extractor_s::check_within_limits(row, col);

        // Datasheet page 36 section 11.2.2.3.
        ta_extractor_c ta(bus);
        // Datasheet page 37 11.2.2.4.
        kgain_c kgain(bus);
        // Datasheet page 39 section 11.2.2.5.3.
        kv_coefficient_c kv_coefficient(bus, row, col);
        // Datasheet page 39 section 11.2.2.5.3
        pix_os_c pix_os(bus, row, col);
        // Datasheet page 40 section 11.2.2.6.1
        cp_compensator_c cp_comp(bus, pattern);
        // Datasheet page 41 section 11.2.2.7.
        patron_extractor_c patron(bus, row, col, pattern);
        // Datasheet page 42 section 11.2.2.7.
        tgc_extractor_c tgc(bus);
        // Datasheet page 42 section 11.2.2.7
        vir_extractor_c vir(bus);
        // Datasheet page 43 section 11.2.2.8
        alpha_comp_extractor_c alpha(bus, row, col);
        // Datasheet page 44 section 11.2.2.9.
        to_extractor_c to(bus);

        extractors = {&ta,  &kgain, &kv_coefficient, &pix_os, &cp_comp, &patron,
                      &tgc, &vir,   &alpha,          &to};

        for (const auto &extractor : extractors) {
            extractor->extract(params);
        }
        return params.To_row_col;
    }

    void
    mlx90640_processor_c::set_reading_pattern(const reading_pattern &pattern) {
        this->pattern = pattern;
    }
} // namespace r2d2::thermal_camera