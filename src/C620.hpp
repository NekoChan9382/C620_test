#ifndef C620_HPP
#define C620_HPP

#include "mbed.h"
#include <array>

namespace dji{

struct C620Data
{
    uint16_t angle;
    int16_t rpm;
    int16_t ampere;
    uint8_t temp;

    void parse(const uint8_t data[8])
    {
        angle = uint16_t(data[0] << 8 | data[1]);
        rpm = int16_t(data[2] << 8 | data[3]);
        ampere = int16_t(data[4] << 8 | data[5]);
        temp = data[6];
    }
};

/// @brief The C620 motor driver class for M3508.
class C620
{
public:
/// @brief Construct a new C620 object
/// @param can_rx CAN receive pin @param can_tx CAN transmit pin
    C620(PinName can_rx, PinName can_tx) : can(*(new CAN(can_rx, can_tx, 1000000))) { memset(output_, 0, sizeof(output_)); }
    C620(CAN& can) : can(can) { memset(output_, 0, sizeof(output_)); }

    ~C620()
    {
        delete &can;
    }

    /// @brief read data from C620
    void read_data()
    {
        CANMessage msg;
        if (can.read(msg); 0x201 <= msg.id && msg.id <= 0x208)
        {
            data_[msg.id - 0x201].parse(msg.data);
        }
    }
    /// @brief set output power
    /// @param current output power 
    /// @param id CAN ID
    void set_output(const int16_t current, const int id)
    {
        if (id < 1 || 8 < id)
        {
            return;
        }
        if (id <= 4)
        {
            output_[0][(id - 1) * 2] = current >> 8 & 0xFF;
            output_[0][(id - 1) * 2 + 1] = current & 0xFF;
        }
        else
        {
            output_[1][(id - 5) * 2] = current >> 8 & 0xFF;
            output_[1][(id - 5) * 2 + 1] = current & 0xFF;
        }
    }
    /// @brief set output power
    /// @param current output power with 8 array
    void set_output(const int16_t current[8])
    {
        for (int i = 0; i < 8; i++)
        {
            set_output(current[i], i + 1);
        }
    }

    /// @brief write output power to C620
    bool write()
    {
        bool is_success[2];
        is_success[0] = can.write(CANMessage(0x200, output_[0], 8));
        is_success[1] = can.write(CANMessage(0x1FF, output_[1], 8));
        
        return is_success[0] && is_success[1];
    }

    int16_t get_current(const int id) const
    {
        if (id < 1 || 8 < id)
        {
            return 0;
        }
        if (id <= 4)
        {
            return output_[0][(id - 1) * 2] << 8 | output_[0][(id - 1) * 2 + 1];
        }
        else
        {
            return output_[1][(id - 5) * 2] << 8 | output_[1][(id - 5) * 2 + 1];
        }
    }

    int16_t get_angle(const int id) const
    {
        if (id < 1 || 8 < id)
        {
            return 0;
        }
        return data_[id - 1].angle;
    }

    int16_t get_rpm(const int id) const
    {
        if (id < 1 || 8 < id)
        {
            return 0;
        }
        return data_[id - 1].rpm;
    }

    int16_t get_ampere(const int id) const
    {
        if (id < 1 || 8 < id)
        {
            return 0;
        }
        return data_[id - 1].ampere;
    }

    uint8_t get_temp(const int id) const
    {
        if (id < 1 || 8 < id)
        {
            return 0;
        }
        return data_[id - 1].temp;
    }

private:
    CAN& can;
    std::array<C620Data, 8> data_;
    uint8_t output_[2][8];
};

} // namespace c620

#endif // C620_HPP