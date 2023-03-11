#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "iostream"

#include <map>
#include <cstdlib> // abs() for integer
using namespace std;

#define SPI_PORT spi0
#define PIN_MISO 0
#define PIN_SS 1
#define PIN_SCLK 2
#define PIN_SOMI 3

// (wrap + 1) x clkdiv = (クロック周波数) / (PWM周波数)
// クロック周波数は125MHz
// PWM周波数は20kHz
#define WRAP_DC 124
#define CLKDIV_DC 50
#define V_MIN 23.75
#define DUTY_DIFF_MAX 70
#define DUTY_MAX 32767

#define MAINMOTOR_NUM 8
#define SUBMOTOR_NUM 4

#define DELAY 1 // 制御分解能(ms)

class DC_motor
{
private:
    const uint8_t PIN_DIRE;
    const uint8_t PIN_PWM;
    uint8_t slice;
    bool chan;
    int duty_prev = 0;

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

DC_motor DCmotors[] = {
    {16, 25},
    {17, 24},
    {21, 22},
    {20, 23},
    {15, 10},
    {14, 11},
    {9, 4},
    {8, 5},
};

class SUB_motor
{
private:
    const uint8_t PIN_A;
    const uint8_t PIN_B;
    uint8_t slice;
    bool chan_A;
    bool chan_B;
    bool chan_pwm;
    int duty_prev = 0;

public:
    SUB_motor(uint8_t PIN_A, uint8_t PIN_B); // コンストラクタ
    void drive(int16_t duty, float volt, bool ifPrint = false);
};

SUB_motor::SUB_motor(uint8_t PIN_A, uint8_t PIN_B)
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

void SUB_motor::drive(int16_t duty, float volt, bool ifPrint)
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
        printf("chan_pwm = %d, chan_A = %d, chan_B = %d\n", chan_pwm, chan_A, chan_B);
    }
    pwm_set_chan_level(slice, chan_pwm, WRAP_DC + 1 - level); // サブチャンネルはHIGHがデフォルト（HIGHの時に静止）なので最大値から指定したlevelを引いた値を引数とする

    if (ifPrint)
        printf("duty_prev:%d, duty:%d, level:%d, Vr1:%f\n", duty_prev, duty, level, volt / 36.3 * 4095.0);

    duty_prev = duty;
}

SUB_motor SUBmotors[] = {
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

public:
    uint raw_val = 0; // ADCを読んだ生の値
    float volt = 0;   // 計算された電圧値
    ADC(uint8_t PIN);
    void read(bool ifPrint = false);
};

ADC::ADC(uint8_t PIN)
    : PIN(PIN)
{
    adc_gpio_init(PIN);
}

void ADC::read(bool ifPrint)
{
    adc_select_input(PIN_TO_INPUT.at(PIN));
    raw_val = adc_read();
    volt = raw_val * 36.3 / 4095.0;
    if (ifPrint)
        printf("raw_val:%d, volt:%f\n", raw_val, volt);
}

ADC Vr1 = ADC(29); //, Vr2 = ADC(28);

int main()
{
    // コンデンサーの充電を待つ
    sleep_ms(1000);

    // モータを回るように調整?
    gpio_init(28);
    gpio_set_dir(28, GPIO_OUT);
    gpio_put(28, 1);

    // シリアル通信初期化
    stdio_init_all();

    // SPI初期化(周波数を4MHzに設定)
    spi_init(SPI_PORT, 4000000);
    // スレーブでSPI通信開始
    spi_set_slave(SPI_PORT, true);
    // SPIピンをアクティブにする
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SS, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCLK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SOMI, GPIO_FUNC_SPI);

    // adc初期化
    adc_init();

    const int motorNum = MAINMOTOR_NUM + SUBMOTOR_NUM;

    while (true)
    {
        Vr1.read(false);
        // Vr2.read();

        // SPI通信の情報を受け取るバッファ
        //| Kモータの方向 (1bit)| モータのDuty (15bit)|を1バイトずつ受け取る。
        uint8_t buf[motorNum * 2];

        // 受信
        spi_read_blocking(SPI_PORT, 0, buf, motorNum * 2);

        int16_t duty[motorNum];

        for (int i = 0; i < motorNum; i++) // 各モータのduty情報は上位ビットから先に送られてくる
        {
            duty[i] = buf[2 * i + 1];
            duty[i] = duty[i] << 8;
            duty[i] |= buf[2 * i];
        }

        for (int i = 0; i < MAINMOTOR_NUM; i++) // メインチャンネル
        {
            DCmotors[i].drive(duty[i], Vr1.volt, 0);
        }
        for (int i = MAINMOTOR_NUM; i < motorNum; i++) // サブチャンネル
        {
            SUBmotors[i - MAINMOTOR_NUM].drive(duty[i], Vr1.volt, i == 8);
        }

        sleep_ms(DELAY);
    }

    return 0;
}
