#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "iostream"
#include "bitset"

#include <map>
#include <cstdlib> // abs() for integer
using namespace std;

#define SPI_PORT spi0
#define PIN_MOSI 0
#define PIN_SS 1
#define PIN_SCLK 2
#define PIN_MISO 3
#define SPI_FREQ 4000000

// (wrap + 1) x clkdiv = (クロック周波数) / (PWM周波数)
// クロック周波数は125MHz
// PWM周波数は20kHz
#define WRAP_DC 124
#define CLKDIV_DC 50
#define V_MIN 22.8        // Cubicの動作可能最低電圧
#define DUTY_DIFF_MAX 100 // モードラが壊れないようDutyの変化を制限
#define DUTY_MAX 32766

#define SOLENOID_TIME 20000 // ソレノイドのON時間(us)

#define MAINMOTOR_NUM 8
#define SOL_SUB_NUM 4 // ソレノイドとサブチャンネルDCモータの数

#define DELAY 1 // 制御分解能(us)

class DC_motor
{
private:
    const uint8_t PIN_DIRE;
    const uint8_t PIN_PWM;
    uint8_t slice;
    bool chan;
    int16_t duty_prev = 0;

public:
    DC_motor(uint8_t PIN_PWM, uint8_t PIN_DIRE);
    void drive(int16_t duty, float volt, bool ifPrint = false);
};

DC_motor::DC_motor(uint8_t PIN_PWM, uint8_t PIN_DIRE)
    : PIN_DIRE(PIN_DIRE), PIN_PWM(PIN_PWM)
{
    gpio_init(PIN_DIRE);
    gpio_set_dir(PIN_DIRE, GPIO_OUT);

    gpio_set_function(PIN_PWM, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(PIN_PWM);

    chan = PIN_PWM % 2;
    pwm_set_wrap(slice, WRAP_DC);
    pwm_set_clkdiv(slice, CLKDIV_DC);
    pwm_set_enabled(slice, true);
    pwm_set_chan_level(slice, chan, 0);
}

void DC_motor::drive(int16_t duty, float volt, bool ifPrint)
{
    // 前回のDutyと同じなら何もしない
    if (duty == duty_prev)
        return;
    // 前回のDutyとの差が大きい場合は、差がDUTY_DIFF_MAX以下になるように調整
    if (abs(duty - duty_prev) > DUTY_DIFF_MAX)
    {
        duty = duty_prev + (duty < duty_prev ? -1 : 1) * DUTY_DIFF_MAX;
    }

    int level = (WRAP_DC + 1) * abs(duty) / DUTY_MAX * V_MIN / volt;
    pwm_set_chan_level(slice, chan, level);
    gpio_put(PIN_DIRE, (duty < 0 ? 1 : 0));

    if (ifPrint)
        printf("duty_prev:%d, duty:%d, level:%d, Vr1:%f\n", duty_prev, duty, level, volt / 36.3 * 4095.0);

    duty_prev = duty;
}

DC_motor motors[] = {
    {16, 25},
    {17, 24},
    {21, 22},
    {20, 23},
    {15, 10},
    {14, 11},
    {9, 4},
    {8, 5}};

class Sub_channel
{
protected:
    const uint8_t PIN_A;
    const uint8_t PIN_B;
    uint8_t slice;
    bool chan_A;
    bool chan_B;
    bool chan_pwm;
    int16_t duty_prev = 0;
    uint64_t time_pre = time_us_64();
    bool init = false;

public:
    int8_t state = -1;                         // -1:初期化, 0:OFF, 1:ON
    Sub_channel(uint8_t PIN_A, uint8_t PIN_B); // コンストラクタ
    void motorDrive(int16_t duty, float volt, bool ifPrint = false);
    void solenoidSwitch(bool state_new, bool ifPrint);
};

Sub_channel::Sub_channel(uint8_t PIN_A, uint8_t PIN_B)
    : PIN_A(PIN_A), PIN_B(PIN_B)
{
    gpio_set_function(PIN_A, GPIO_FUNC_PWM);
    gpio_set_function(PIN_B, GPIO_FUNC_PWM);
    chan_A = PIN_A % 2;
    chan_B = PIN_B % 2;

    slice = pwm_gpio_to_slice_num(PIN_A);
    pwm_set_wrap(slice, WRAP_DC);
    pwm_set_clkdiv(slice, CLKDIV_DC);
    pwm_set_enabled(slice, true);
    pwm_set_chan_level(slice, chan_A, WRAP_DC + 1);
    pwm_set_chan_level(slice, chan_B, WRAP_DC + 1);
}

void Sub_channel::motorDrive(int16_t duty, float volt, bool ifPrint)
{
    // 前回のDutyと同じなら何もしない
    if (duty == duty_prev)
        return;
    // 前回のDutyとの差が大きい場合は、差がDUTY_DIFF_MAX以下になるように調整
    if (abs(duty - duty_prev) > DUTY_DIFF_MAX)
    {
        duty = duty_prev + (duty < duty_prev ? -1 : 1) * DUTY_DIFF_MAX;
    }
    // サブチャンネルはHIGHがデフォルト（HIGHの時に静止）なので，duty比は最大値1から指定した値を引いたものとする
    int level = (WRAP_DC + 1.0) * (1.0 - (double)abs(duty) / DUTY_MAX * V_MIN / volt);

    // dutyの正負が逆転していたらHIGHを与えるピンを変更
    // bool chan_pwm;
    if (duty * duty_prev <= 0)
    {
        if (duty > 0) // BがHIGHでモーター向かってCW
        {
            chan_pwm = chan_A;
            pwm_set_chan_level(slice, chan_B, WRAP_DC + 1); // chan_BをHIGH
        }
        else if (duty < 0)
        {
            chan_pwm = chan_B;
            pwm_set_chan_level(slice, chan_A, WRAP_DC + 1); // chan_AをHIGH
        }
        // printf("chan_pwm = %d, chan_A = %d, chan_B = %d\n", chan_pwm, chan_A, chan_B);
    }
    pwm_set_chan_level(slice, chan_pwm, level);

    if (ifPrint)
        printf("duty_prev:%d, duty:%d, level:%d, Vr1:%f\n", duty_prev, duty, level, volt / 36.3 * 4095.0);

    duty_prev = duty;
}

void Sub_channel::solenoidSwitch(bool state_new, bool ifPrint)
{
    uint64_t time_now = time_us_64();
    // SOLENOID_TIME(us)以下の間は何もしない
    if (time_now - time_pre < SOLENOID_TIME)
        return;

    if (state == state_new)
    {
        // 両方HIGHにする
        pwm_set_chan_level(slice, chan_A, WRAP_DC + 1);
        pwm_set_chan_level(slice, chan_B, WRAP_DC + 1);
        // printf("HIGH HIGH  ");
    }
    else
    {
        // 一方LOWにする
        if (state_new)
        {
            pwm_set_chan_level(slice, chan_A, WRAP_DC + 1);
            pwm_set_chan_level(slice, chan_B, 0);
            // printf("HIGH LOW  ");
        }
        else
        {
            pwm_set_chan_level(slice, chan_A, 0);
            pwm_set_chan_level(slice, chan_B, WRAP_DC + 1);
            // printf("LOW HIGH  ");
        }
        time_pre = time_now;
        state = state_new;
    }
    if (ifPrint)
        printf("state:%d, time:%d\n", state, time_now);
}

// class Solenoid
// {
// private:
//     const uint8_t PIN_A;
//     const uint8_t PIN_B;
//     uint64_t time_pre = time_us_64();
//     bool init = false;

// public:
//     Solenoid(uint8_t PIN_A, uint8_t PIN_B);
//     void begin();
//     void Switch(bool state, bool ifPrint = false);
//     int8_t state = -1; // -1:初期化, 0:OFF, 1:ON
// };

// // 最初の1回だけ初期化する関数
// void Solenoid::begin()
// {
//     if (init == false)
//     {
//         // 初期化
//         gpio_init(PIN_A);
//         gpio_init(PIN_B);
//         gpio_set_dir(PIN_A, GPIO_OUT);
//         gpio_set_dir(PIN_B, GPIO_OUT);
//         gpio_put(PIN_A, 1);
//         gpio_put(PIN_B, 1);
//         init = true;
//     }
// }

Sub_channel Sub_channel[] = {
    {26, 27},
    {19, 18},
    {13, 12},
    {7, 6}};

class ADC
{
private:
    const uint8_t PIN;
    static inline const map<uint8_t, uint8_t> PIN_TO_INPUT = {
        {26, 0},
        {27, 1},
        {28, 2},
        {29, 3}};
    float volt_prev = 0;

public:
    uint raw_val = 0; // ADCを読んだ生の値
    float volt = 0;   // 計算された電圧値
    ADC(uint8_t PIN);
    void read(bool ifPrint = false, bool ifFilter = false);
};

ADC::ADC(uint8_t PIN)
    : PIN(PIN)
{
    adc_gpio_init(PIN);
}

void ADC::read(bool ifPrint, bool ifFilter)
{
    adc_select_input(PIN_TO_INPUT.at(PIN));
    raw_val = adc_read();

    if (ifFilter)
        volt = volt_prev * 0.99 + (raw_val * 39.6 / 4096.0) * 0.01;
    else
        volt = raw_val * 39.6 / 4096.0;
    if (volt < V_MIN)
        volt = V_MIN;
    volt_prev = volt;

    if (ifPrint)
        printf("raw_val:%d, volt:%f\n", raw_val, volt);
}

ADC Vr1 = ADC(29);
ADC Vr2 = ADC(28);

int main()
{
    // コンデンサーの充電を待つ
    ///*
    sleep_ms(1000);

    // シリアル通信初期化
    stdio_init_all();

    // SPI初期化(周波数を4MHzに設定)
    spi_init(SPI_PORT, SPI_FREQ);
    // スレーブでSPI通信開始
    spi_set_slave(SPI_PORT, true);
    // SPIピンをアクティブにする
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SS, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCLK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);

    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // adc初期化
    adc_init();
    ///*
    while (true)
    {
        Vr2.read(false, false); // 起動時にVr2の値が閾値よりも高ければその後の負荷の増加などで止まらないようにする。
        if (Vr2.volt > V_MIN)
        {
            sleep_ms(100);
            gpio_init(28);
            gpio_set_dir(28, GPIO_OUT);
            gpio_put(28, 1);
            break;
        }
        sleep_ms(1);
    }
    //*/

    const int motor_num = MAINMOTOR_NUM + SOL_SUB_NUM;
    uint8_t buf[motor_num * 2];       // SPI通信の情報を受け取るバッファ
    const uint8_t request_buf = 0xFF; // マスターへの送信要求バッファ "11111111"

    int16_t duty[motor_num];

    Vr1.read(false, false);
    int cnt = 0;

    while (true)
    {
        Vr1.read(false);
        if (Vr1.volt < V_MIN)
            Vr1.volt = V_MIN;

        spi_write_blocking(SPI_PORT, &request_buf, 1); // リクエスト送信

        /*
            for(int i=0;i<motor_num;i++){
            std::cout << std::bitset<16>(duty[i]) << ",";
            }
            std::cout << "\n";
            // std::cout << std::bitset<16>(duty[0]) << ":" << std::bitset<8>(buf[1]) << "," << std::bitset<8>(buf[0]) << "\n";
        */

        spi_read_blocking(SPI_PORT, 0, buf, motor_num * 2); // データ受信

        for (int i = 0; i < motor_num; i++)
        {
            duty[i] = buf[2 * i + 1];
            duty[i] = duty[i] << 8;
            duty[i] |= buf[2 * i];
        }
        for (int i = 0; i < MAINMOTOR_NUM; i++)
        {
            // メインチャンネルDCモータの制御
            motors[i].drive(duty[i], Vr1.volt, false);
        }
        for (int i = MAINMOTOR_NUM; i < motor_num; i++)
        {
            if (abs(duty[i]) != DUTY_MAX + 1)
            {
                // サブチャンネルDCモータの制御
                Sub_channel[i - MAINMOTOR_NUM].motorDrive(duty[i], Vr1.volt, false);
            }
            else
            {
                //ソレノイドの制御;
                Sub_channel[i - MAINMOTOR_NUM].solenoidSwitch(duty[i] > 0, false);
            }
        }

        cnt++;
        if (cnt % 100 == 0)
        {
            Vr1.read(false, true);
            cnt = 0;
        }

        sleep_us(DELAY);
    }

    return 0;
}
