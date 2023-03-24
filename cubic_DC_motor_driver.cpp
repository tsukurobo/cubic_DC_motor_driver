#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#include <map>
#include <cstdlib> // abs() for integer
using namespace std;

#define SPI_PORT spi0
#define PIN_SIMO 0
#define PIN_SS 1
#define PIN_SCLK 2
#define PIN_SOMI 3
#define SPI_FREQ 400000

// (wrap + 1) x clkdiv = (クロック周波数) / (PWM周波数)
// クロック周波数は125MHz
// PWM周波数は20kHz
#define WRAP_DC 124
#define CLKDIV_DC 50
#define V_MIN 23.75
#define DUTY_DIFF_MAX 70
#define DUTY_MAX 32766

#define SOLENOID_TIME 10000 // ソレノイドのON時間(us)

#define MAINMOTOR_NUM 8
#define SOL_SUB_NUM 4 // ソレノイドとサブチャンネルDCモータの数

#define DELAY 1 // 制御分解能(ms)

class DC_motor{
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

void DC_motor::drive(int16_t duty, float volt, bool ifPrint){
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

class SUB_motor{
private:
    const uint8_t PIN_A;
    const uint8_t PIN_B;
    uint8_t slice;
    bool chan_A;
    bool chan_B;
    bool chan_pwm;
    int16_t duty_prev = 0;

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

void SUB_motor::drive(int16_t duty, float volt, bool ifPrint){
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

class ADC{
private:
    const uint8_t PIN;
    static inline const map<uint8_t, uint8_t> PIN_TO_INPUT = {
        {26, 0},
        {27, 1},
        {28, 2},
        {29, 3}
    };
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

void ADC::read(bool ifPrint){
    adc_select_input(PIN_TO_INPUT.at(PIN));
    raw_val = adc_read();
    volt = raw_val * 36.3 / 4095.0;
    if (ifPrint)
        printf("raw_val:%d, volt:%f\n", raw_val, volt);
}

ADC Vr1 = ADC(29); //, Vr2 = ADC(28);

class Solenoid {
private:
    const uint8_t PIN_A;
    const uint8_t PIN_B;
    uint64_t time_pre = time_us_64();
    bool init = false;
public:
    Solenoid(uint8_t PIN_A, uint8_t PIN_B);
    void begin();
    void Switch(bool state, bool ifPrint = false);
    int8_t state = -1; // -1:初期化, 0:OFF, 1:ON
};

Solenoid::Solenoid(uint8_t PIN_A, uint8_t PIN_B)
    : PIN_A(PIN_A), PIN_B(PIN_B)
{
}

// 最初の1回だけ初期化する関数
void Solenoid::begin() {
    if (init == false) {
        // 初期化
        gpio_init(PIN_A);
        gpio_init(PIN_B);
        gpio_set_dir(PIN_A, GPIO_OUT);
        gpio_set_dir(PIN_B, GPIO_OUT);
        gpio_put(PIN_A, 1);
        gpio_put(PIN_B, 1);
        init = true;
    }
}

void Solenoid::Switch(bool state_new, bool ifPrint) {
    uint64_t time_now = time_us_64();
    // SOLENOID_TIME(us)以下の間は何もしない
    if (time_now - time_pre < SOLENOID_TIME) return;

    if (state == state_new) {
        // 両方HIGHにする
        gpio_put(PIN_A, 1);
        gpio_put(PIN_B, 1);
        //printf("HIGH HIGH  ");
    }
    else {
        // 一方LOWにする
        if (state_new) {
            gpio_put(PIN_A, 1);
            gpio_put(PIN_B, 0);
            //printf("HIGH LOW  ");
        }
        else {
            gpio_put(PIN_A, 0);
            gpio_put(PIN_B, 1);
            //printf("LOW HIGH  ");
        }
        time_pre = time_now;
        state = state_new;
    }
    if (ifPrint)
        printf("state:%d, time:%d\n", state, time_now);
}

Solenoid solenoid[] = {
    {26, 27},
    {19, 18},
    {13, 12},
    { 7,  6}
};

//SPI通信の情報を受け取るバッファ
uint8_t buf[MAINMOTOR_NUM*2+SOL_SUB_NUM*2];

void spi_receive(uint gpio, uint32_t events) {
    if (gpio == PIN_SS && events == GPIO_IRQ_EDGE_FALL) {
        gpio_set_irq_enabled(PIN_SS, GPIO_IRQ_EDGE_FALL, false);

        gpio_set_function(PIN_SOMI, GPIO_FUNC_SPI);
        gpio_set_function(PIN_SS,   GPIO_FUNC_SPI);

        spi_read_blocking(SPI_PORT, 0, buf, MAINMOTOR_NUM*2+SOL_SUB_NUM*2);

        gpio_init(PIN_SOMI);
        gpio_set_dir(PIN_SOMI, GPIO_IN);
        gpio_init(PIN_SS);
        gpio_set_dir(PIN_SS, GPIO_IN);
        gpio_pull_up(PIN_SS);

        gpio_set_irq_enabled(PIN_SS, GPIO_IRQ_EDGE_FALL, true);
    }
}

int main()
{
    // SPIピンをアクティブにする
    /*
        スレーブ動作の際にSSがHIGHのときもMISOがLOWになってしまうSPIライブラリのバグがあるため，
        MISOとSSをGPIOピンとして初期化し，SSの立ち下がりエッジで割り込み処理をしSPI通信を行う
    */
    gpio_set_function(PIN_SIMO, GPIO_FUNC_SPI);
    //gpio_set_function(PIN_SS,   GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCLK, GPIO_FUNC_SPI);
    //gpio_set_function(PIN_SOMI, GPIO_FUNC_SPI);
    gpio_init(PIN_SS);
    gpio_set_dir(PIN_SS, GPIO_IN);
    gpio_pull_up(PIN_SS);
    gpio_init(PIN_SOMI);
    gpio_set_dir(PIN_SOMI, GPIO_IN);

    // SPI初期化(周波数を4MHzに設定)
    spi_init(SPI_PORT, SPI_FREQ);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    // スレーブでSPI通信開始
    spi_set_slave(SPI_PORT, true);

    gpio_set_irq_enabled_with_callback(PIN_SS, GPIO_IRQ_EDGE_FALL, true, spi_receive);
    
    // コンデンサーの充電を待つ
    sleep_ms(1000);

    // モータを回るように調整?
    gpio_init(28);
    gpio_set_dir(28, GPIO_OUT);
    gpio_put(28, 1);

    // シリアル通信初期化
    stdio_init_all();
    
    // adc初期化
    adc_init();

    const int motorNum = MAINMOTOR_NUM + SOL_SUB_NUM;

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
            if (abs(duty[i]) != DUTY_MAX+1) {
                // サブチャンネルDCモータの制御
                SUBmotors[i - MAINMOTOR_NUM].drive(duty[i], Vr1.volt, i == 8);
            }
            else {
                // ソレノイドの制御
                solenoid[i].begin();
                solenoid[i].Switch(duty[i] > 0, false);
            }
        }

        sleep_ms(DELAY);
    }

    return 0;
}
