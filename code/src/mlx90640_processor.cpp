#include <mlx90640_processor.hpp>

#include <alpha.hpp>
#include <kta.hpp>
#include <kv.hpp>
#include <pix_os_ref.hpp>

#include <gain_cp.hpp>
#include <kgain.hpp>
#include <off_ta_vdd_cp.hpp>
#include <resolution.hpp>
#include <ta.hpp>
#include <vdd_var.hpp>

#include <gain_comp.hpp>
#include <ir_gradient_comp.hpp>
#include <pix_os.hpp>
#include <to.hpp>
#include <vir_compensator.hpp>

#include <alpha_corr.hpp>
#include <alpha_cp.hpp>
#include <ee_corner_temp.hpp>
#include <ee_gain.hpp>
#include <ee_ksta.hpp>
#include <ee_ksto.hpp>
#include <ee_kta_cp.hpp>
#include <ee_kv_cp.hpp>
#include <ee_off_cp.hpp>
#include <ee_resolution.hpp>
#include <ee_ta.hpp>
#include <ee_tgc_extractor.hpp>
#include <ee_vdd.hpp>

namespace r2d2::thermal_camera {

    mlx90640_processor_c::mlx90640_processor_c(mlx90640_i2c_c &bus,
                                               float emissivity)
        : bus(bus), params{}, pixels{{}} {
        params.emissivity = emissivity;

        hwlib::cout << "mlx90640 initializing...\n";

        // Datasheet section 11.1.3
        pix_os_ref_c pix_offset(bus, params);
        // Datasheet section 11.1.4
        alpha_c alpha(bus, params);
        // Datasheet section 11.1.5
        kv_c kv(bus, params);
        // Datasheet section 11.1.6
        kta_c kta(bus, params);

        lookupables = {&pix_offset, &alpha, &kv, &kta};

        for (const auto &l : lookupables) {
            init_table(*l);
        }

        // Datasheet section 11.1.1
        ee_vdd_c ee_vdd(bus, params);
        // Datasheet section 11.1.2
        ee_ta_c ee_ta(bus, params);
        // Datasheet section 11.1.7
        ee_gain_c ee_gain(bus, params);
        // Datasheet section 11.1.8
        ee_ksta_c ksta(bus, params);
        // Datasheet section 11.1.9
        ee_corner_temp_c ct(bus, params);
        // Datasheet section 11.1.10
        ee_ksto_c ksto(bus, params);
        // Datasheet section 11.1.11
        alpha_corr_c alpha_corr(bus, params);
        // Datasheet section 11.1.12
        alpha_cp_c alpha_cp(bus, params);
        // Datasheet section 11.1.13
        ee_off_cp_c cp_offset(bus, params);
        // Datasheet section 11.1.14
        ee_kv_cp_c kv_cp(bus, params);
        // Datasheet section 11.1.15
        ee_kta_cp_c kta_cp(bus, params);
        // Datasheet section 11.1.16
        ee_tgc_extractor_c tgc(bus, params);
        // Datasheet section 11.1.17
        ee_resolution_c resolution(bus, params);

        static_vars = {&ee_vdd, &ee_ta,  &ee_gain,    &tgc,       &ksta,
                       &ct,     &ksto,   &alpha_corr, &alpha_cp,  &cp_offset,
                       &kv_cp,  &kta_cp, &tgc,        &resolution};
        // Sets all EEPROM data in mlx params
        for (const auto &static_var : static_vars) {
            static_var->extract();
        }

        hwlib::cout << "mlx90640 initialized\n";
    }

    void mlx90640_processor_c::init_table(lookupable_c &table) {
        for (unsigned int i = 1; i <= pixels.size(); i++) {        // 24 rows
            for (unsigned int j = 1; j <= pixels[i].size(); j++) { // 32 cols
                table.calculate_pixel(i, j);
            }
        }
    }

    void mlx90640_processor_c::calculate_pixel_value(
        pixel_manipulator_c &manipulator) {
        for (unsigned int i = 1; i <= pixels.size(); i++) {        // 24 rows
            for (unsigned int j = 1; j <= pixels[i].size(); j++) { // 32 cols
                manipulator.calculate_pixel(i, j);
            }
        }
    }

    // Pipeline
    void mlx90640_processor_c::set_frame() {
        // Datasheet section 11.2.2.1
        resolution_c res(bus, params);
        // Datasheet section 11.2.2.2
        vdd_var_c vdd(bus, params);
        // Datasheet section 11.2.2.3
        ta_c ta(bus, params);
        // Datasheet section 11.2.2.4
        kgain_c kgain(bus, params);

        dynamic_vars = {&res, &vdd, &ta, &kgain};

        for (int i = 0; i < 4; i++) {
            dynamic_vars[i]->re_calculate();
        }

        // Datasheet section 11.2.2.5.1
        gain_comp_c gain(bus, params, pixels);
        /* Datasheet section 11.2.2.5.2
         This calculation already has been done by pix_os_ref object.*/
        // Datasheet section 11.2.2.5.3
        pix_os_c pix_os(params, pixels, *lookupables[KTA], *lookupables[KV],
                        *lookupables[PIX_OS_REF]);
        // Datasheet section 11.2.2.5.4
        vir_compensator compensator(params, pixels);

        pixel_calculators = {&gain, &pix_os, &compensator};

        for (int i = 0; i < 3; i++) {
            calculate_pixel_value(*pixel_calculators[i]);
        }

        // Datasheet section 11.2.2.6.1
        gain_cp_c gain_cp(bus, params);
        // Datasheet section 11.2.2.6.2
        off_ta_vdd_cp_c ta_vdd_cp_offset(bus, params, pattern);

        dynamic_vars = {&gain_cp, &ta_vdd_cp_offset};

        for (int i = 0; i < 2; i++) {
            dynamic_vars[i]->re_calculate();
        }

        // Datasheet section 11.2.2.7
        ir_gradient_comp ir_gradient(params, pixels, pattern);
        // Datasheet section 11.2.2.8
        to_c to(params, pixels, pattern, *lookupables[ALPHA]);

        pixel_calculators = {&ir_gradient, &to};

        for (int i = 0; i < 2; i++) {
            calculate_pixel_value(*pixel_calculators[i]);
        }
    }

    std::array<std::array<float, 32>, 24> &mlx90640_processor_c::get_frame() {
        return pixels;
    }

    void
    mlx90640_processor_c::set_reading_pattern(const reading_pattern &pattern) {
        this->pattern = pattern;
    }
} // namespace r2d2::thermal_camera