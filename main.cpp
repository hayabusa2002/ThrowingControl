#include "mbed.h"
#include <chem30338.h>
#include <Encoder.h>
#include <cstdio>

constexpr float pi = 3.1415f;
constexpr float tau = 2 * pi;

constexpr float radius_m = 0.016f / 2.0;
constexpr float stroke_m = 0.40f;

constexpr float position_to_throw_m = 0.75f * stroke_m;
constexpr float position_at_top_speed_m = position_to_throw_m * 0.80f;
constexpr float position_to_stop_m = 0.9 * stroke_m;
constexpr float velocity_top = 0.80f; // meter per sec
constexpr float ang_vel_top_rps = velocity_top / (radius_m * tau); // rotation per sec

constexpr float slope_motor_f = 6.7385f / (radius_m * tau);
constexpr float intercept_motor_f = - 0.12f / (radius_m * tau);

float time_acceleration_s, time_at_top_speed_s, time_deceleration_s, time_whole_s;
void calculate_timing();

int prev_enc_ticks;

constexpr float duty_limit = 0.80f;

constexpr PinName pin_enc_a_ch = PA_0;
constexpr PinName pin_enc_b_ch = PA_1;
constexpr int enc_resolution = 800;
constexpr float duration_s = 0.005; // 5 ms
constexpr chrono::milliseconds duration_ms = 5ms;
mext::Encoder<int> enc(pin_enc_a_ch, pin_enc_b_ch);

PwmOut pwm1(PB_14);
PwmOut pwm2(PB_13);
Timer timer;
Ticker ticker;
BufferedSerial ser(CONSOLE_TX, CONSOLE_RX);

using namespace chem30338;
constexpr pid_params params = pid_params_builder(p_gain{0.8f}, i_time{0.4f}, d_time{0.1f})
        .alpha(0.1f).beta(0.5f).gamma(0.5f).params();
pid_bilinear pid(params, duration_s);

int data_count;
bool is_finished;
int data_size;
float *data_duty, *data_ang_vel_output, *data_ang_vel_target, *data_ang_vel_measured;
void allocate_memory_for_data();
void contain_data(float, float, float, float);
void print_data();

float get_measured_ang_vel();
float get_target_ang_vel(float);
void get_output_duty();
void output_motor();

template<class T> 
constexpr const T &clamp(const T &v, const T &lo, const T &hi) {
    return (v < lo) ? lo : ((hi < v) ? hi : v);
}

bool is_running;

int main() {
    printf("\r\nstart? : y\n");

    calculate_timing();
    allocate_memory_for_data();

    pwm1.period_us(50);
    pwm2.period_us(50);
    
    pwm1.write(0.0f);
    pwm2.write(0.0f);

    while (true) {
        char c;
        ser.read(&c, 1);
        if (c == 'y') {
            break;
        }
    }

    //開始
    timer.start();
    timer.reset();
    ticker.attach(&output_motor, duration_ms);

    is_running = true;

    while (is_running) {}

    //停止
    printf("\r\nfinished!\r\n");
    ticker.detach();
    timer.stop();
    pwm1.write(0.0f);
    pwm2.write(0.0f);

    print_data();

    free(data_duty);
    free(data_ang_vel_target);
    free(data_ang_vel_measured);
    free(data_ang_vel_output);

}

void calculate_timing() {
    
}

void output_motor() {
    float present_time_s = timer.read_ms() / 1000;
    float ang_vel_measured_rps = get_measured_ang_vel();
    float ang_vel_target_rps = get_target_ang_vel(present_time_s);

    float ang_vel_output_rps = pid.update(ang_vel_target_rps, ang_vel_measured_rps);
    float duty_output = (ang_vel_output_rps - intercept_motor_f) / slope_motor_f;
    duty_output = clamp<float>(duty_output, 0, duty_limit);

    pwm1.write(duty_output);

    contain_data(duty_output, ang_vel_target_rps, ang_vel_output_rps, ang_vel_measured_rps);
}

float get_measured_ang_vel() {
    float ang_vel_measured_rps;
    int enc_ticks = enc.ticks();
    float delta_ticks = static_cast<float>(enc_ticks - prev_enc_ticks) / duration_s;

    ang_vel_measured_rps = delta_ticks / duration_s;

    return ang_vel_measured_rps; //単位 rotation per sec
}

float get_target_ang_vel(float present_sec) {
    float ang_vel_target_rps;
    if (present_sec <= time_acceleration_s) {
        ang_vel_target_rps = ang_vel_top_rps * (present_sec / time_acceleration_s);
    } else if (present_sec <= time_acceleration_s + time_at_top_speed_s) {
        ang_vel_target_rps = ang_vel_top_rps;
    } else if (present_sec <= time_whole_s) {
        ang_vel_target_rps = ang_vel_top_rps * ((time_whole_s - present_sec) / (time_deceleration_s));
    } else {
        ang_vel_target_rps = 0.0f;
        is_running = false;
    }

    return ang_vel_target_rps;
}

void allocate_memory_for_data() {
    data_size = static_cast<int>(time_whole_s / (1000 * duration_s)) + 1;
    data_count = 0;
    is_finished = false;
    data_duty = static_cast<float*>(malloc(data_size * sizeof(float)));
    data_ang_vel_output = static_cast<float*>(malloc(data_size * sizeof(float)));
    data_ang_vel_target = static_cast<float*>(malloc(data_size * sizeof(float)));
    data_ang_vel_measured = static_cast<float*>(malloc(data_size * sizeof(float)));
    if (data_duty == NULL || data_ang_vel_output == NULL || data_ang_vel_target == NULL || data_ang_vel_measured == NULL) {
        printf("malloc failed\r\n");
        is_finished = true;
    }
}

void contain_data(float duty, float vel_output, float vel_target, float vel_measured) {
    if (!is_finished) {
        data_duty[data_count] = duty;
        data_ang_vel_output[data_count] = vel_output;
        data_ang_vel_target[data_count] = vel_target;
        data_ang_vel_measured[data_count] = vel_measured;

        ++data_count;

        if (data_count == data_size - 1) {
            is_finished = true;
        }
    }
}

void print_data() {
    printf("\r\ntimer_s, duty, vel_output, vel_target, vel_measured\r\n");
    for (int i = 0; i < data_count; ++i) {
        printf("\r\n%.9f, %.9f, %.9f, %.9f, %.9f", duration_s * data_count, data_duty[i], data_ang_vel_output[i], data_ang_vel_target[i], data_ang_vel_measured[i]);
    }
}
