#include <mbed.h>
#include <array>
#include "PID_new.hpp"
#include "C620.hpp"

constexpr int motor_id = 4;

int main()
{
    dji::C620 c620(PA_11, PA_12);
    int output_power = 4000;

    BufferedSerial serial(USBTX, USBRX, 115200); //シリアル初期化
    DigitalIn button(BUTTON1,PullUp);
    int16_t rpm = 0;
    int16_t rpm2 = 0;
    int16_t ampere = 0;
    int pos = 0;
    int pos2 = 0;
    int rpm_goal = 0;
    int ang_goal = 0;
    PidGain gain = {2.0, 0.3, 0};
    PidGain gain_pos = {0.8, 0.0001, 0.1};
    Pid pid({gain, -7500, 7500});


    while(1)
    {
        auto time = HighResClock::now();
        static auto pre = time;

        if (button == 0)
        {
            rpm_goal = output_power;
            ang_goal = -226 * 20 *1000;
        }
        else
        {
            rpm_goal = 0;
            ang_goal = 0;
        }
        c620.read_data();
        rpm = c620.get_rpm(motor_id);
        // rpm2 = c620.get_rpm(1);
        ampere = c620.get_ampere(5);
        // pos = c620.get_angle(5);

        
        if (time - pre > 10ms)
        {
            constexpr float rmp_to_rad = 2 * M_PI / 60;
            float rad = rpm * rmp_to_rad;
            float rad2 = rpm2 * rmp_to_rad;
            pos += rad;
            pos2 += rad2;
            // c620.set_output(pid.calc(ang_goal, pos, 0.01), 5);
            // c620.set_output(pid.calc(-ang_goal, pos, 0.01), 2);
            
            c620.set_output(pid.calc(rpm_goal, rpm, 0.01), motor_id);
            printf("write: %d\n", c620.write());
            // printf("rpm: %d, ampere: %d, output: %d, dps: %f, goal: %d, ang: %d\n", rpm, ampere, c620.get_current(5), rpm * rmp_to_rad, rpm_goal, pos);
            // printf("ang: %d\n", pos);
            pre = time;
        }
    }
}