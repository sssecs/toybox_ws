#pragma once

#include <libserial/SerialPort.h>
#include <iostream>

class Stm32Comms
{
public:

    void connect(const std::string &serial_device, int16_t timeout_ms);

    bool check_connected();

    void disconnect();

    void send_rad_velo(double velo_l, double velo_r);

    void read_rad_velo_pos(double &velo_l, double &velo_r,double &pos_l, double &pos_r);

    

private:

    static const int output_array_length_ = 8;
    static const int input_array_length_ = 24;

    const double encoder_counts_pre_round_ = 60000;
    const double ticks_pre_sec_ = 100;



    int16_t timeout_ms_;
    LibSerial::SerialPort serial_port_;
    char output_raw_[output_array_length_];
    std::string input_raw_;

    int wheel_l_round_num_ = 0;
    int wheel_r_round_num_ = 0;

    int pos_l_history_ = encoder_counts_pre_round_ + 1;
    int pos_r_history_ = encoder_counts_pre_round_ + 1;
    

};