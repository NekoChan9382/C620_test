#include <mbed.h>
#include <array>
#include "PID_new.hpp"
#include "C620.hpp"


int main()
{
    dji::C620 c620(PA_11, PA_12);
    int output_power = 3000;

    BufferedSerial serial(USBTX, USBRX, 115200); //シリアル初期化
    DigitalIn button(BUTTON1,PullUp);
    int16_t rpm = 0;
    int16_t ampere = 0;
    int rpm_goal = 0;
    PidGain gain = {2.0, 0.3, 0};
    Pid pid({gain, -5000, 5000});

    while(1)
    {
        auto time = HighResClock::now();
        static auto pre = time;

        if (button == 0)
        {
            rpm_goal = output_power;
        }
        else
        {
            rpm_goal = 0;
        }
        c620.read_data();
        rpm = c620.get_rpm(1);
        ampere = c620.get_ampere(1);
        
        if (time - pre > 10ms)
        {
            c620.set_output(pid.calc(rpm_goal, rpm, 0.01), 1);
            c620.write();
            printf("rpm: %d, ampere: %d, output: %d\n", rpm, ampere, c620.get_current(1));
            pre = time;
        }
    }
}