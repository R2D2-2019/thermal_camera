#include <mlx90640_processor.hpp>

namespace r2d2::thermal_camera {

    mlx90640_processor_c::mlx90640_processor_c(i2c::i2c_bus_c &bus,
                                               const uint8_t address)
        : bus(bus), address(address) {
    }

    uint16_t
    mlx90640_processor_c::read_register(const uint16_t internal_address) const {
        uint8_t raw_data[2];
        bus.read(address, raw_data, 2, internal_address, 2);
        return static_cast<uint16_t>((raw_data[1] << 8) | raw_data[0]);
    }

    void mlx90640_processor_c::write_register(const uint16_t internal_address,
                                              const uint16_t data) const {
        uint8_t array_data[] = {static_cast<uint8_t>(data),
                                static_cast<uint8_t>(data >> 8)};
        bus.write(address, array_data, 2, internal_address, 2);
    }

    int16_t mlx90640_processor_c::get_delta_V() {
        int16_t Kvdd = (read_register(EE_VDD_PIX) & 0xFF00) / 256;
        if (Kvdd > 127) {
            Kvdd -= 256;
        }
        Kvdd *= 32;
        int16_t VDD25 = (read_register(EE_VDD_PIX) - 256) * 32 - 8192;
        int16_t ram_vdd_pix = read_register(RAM_VDD_PIX);
        if (ram_vdd_pix > 32767) {
            ram_vdd_pix -= 65536;
        }
        return static_cast<int16_t>((ram_vdd_pix - VDD25) / Kvdd);
    }

    int16_t mlx90640_processor_c::get_gain() {
        int16_t gain = read_register(EE_GAIN);
        if (gain > 32767) {
            gain -= 65536;
        }
        int16_t ram_gain = read_register(RAM_GAIN);
        if (ram_gain > 32767) {
            ram_gain -= 65536;
        }
        return static_cast<int16_t>(gain / ram_gain);
    }

} // namespace r2d2::thermal_camera