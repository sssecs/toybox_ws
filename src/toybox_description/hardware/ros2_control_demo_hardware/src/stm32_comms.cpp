#include "ros2_control_demo_hardware/stm32_comms.h"
#define PI           3.14159265358979323846



void Stm32Comms::connect(const std::string &serial_device, int16_t timeout_ms)
{
    this->timeout_ms_ = timeout_ms;
    try
    {
        // Open the Serial Port at the desired hardware port.
        serial_port_.Open(serial_device) ;
    }
    catch (const LibSerial::OpenFailed&)
    {
        std::cerr << "The serial port did not open correctly." << std::endl ;
    }

    // Set the baud rate of the serial port.
    serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200) ;

    // Set the number of data bits.
    serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8) ;

    // Turn off hardware flow control.
    serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE) ;

    // Disable parity.
    serial_port_.SetParity(LibSerial::Parity::PARITY_NONE) ;
    
    // Set the number of stop bits.
    serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1) ;
    
}

bool Stm32Comms::check_connected()
{
    return this->serial_port_.IsOpen();
}

void Stm32Comms::disconnect()
{
    serial_port_.Close();
}


void Stm32Comms::send_rad_velo(double velo_l, double velo_r)
{
    int count_l,count_r;
    count_l = (int)(velo_l/(2*PI)*this->encoder_counts_pre_round_/this->ticks_pre_sec_);
    count_r = (int)(velo_r/(2*PI)*this->encoder_counts_pre_round_/this->ticks_pre_sec_);

    this->output_raw_[0]='{'; //frame head 0x7B //帧头0X7B
    this->output_raw_[1] = count_l/10000; 
    this->output_raw_[2] = (count_l-count_l/10000*10000)/100; 
    this->output_raw_[3] = count_l%100; 
    this->output_raw_[4] = count_r/10000; 
    this->output_raw_[5] = (count_r-count_r/10000*10000)/100; 
    this->output_raw_[6] = count_r%100; 

    this->output_raw_[7]='}'; //frame tail 0x7D //帧尾0X7D

    for (int i=0;i<this->output_array_length_;i++)
    {
        this->serial_port_.WriteByte(this->output_raw_[i]);
    }
}


void Stm32Comms::read_rad_velo_pos(double &velo_l, double &velo_r,double &pos_l, double &pos_r)
{
    int velo_l_raw, velo_r_raw, pos_l_raw, pos_r_raw;
    try
    {   
        
        this->serial_port_.ReadLine(this->input_raw_, '}', this->timeout_ms_);
        if (this->input_raw_.size() ==15)
        {
 	    velo_l_raw = input_raw_[2]*10000 + input_raw_[3]*100 + input_raw_[4];
	    velo_r_raw = input_raw_[5]*10000 + input_raw_[6]*100 + input_raw_[7];

        pos_l_raw = input_raw_[8]*10000 + input_raw_[9]*100 + input_raw_[10];
	    pos_r_raw = input_raw_[11]*10000 + input_raw_[12]*100 + input_raw_[13];
        
        }
        velo_l = (double)velo_l_raw / this->encoder_counts_pre_round_ *2*PI*this->ticks_pre_sec_;
        velo_r = (double)velo_r_raw / this->encoder_counts_pre_round_ *2*PI*this->ticks_pre_sec_;

        if (abs(this->pos_l_history_) > this->encoder_counts_pre_round_ || abs(this->pos_r_history_) > this->encoder_counts_pre_round_)
        {
            this->pos_l_history_ = pos_l_raw;
            this->pos_r_history_ = pos_r_raw;
        }


        if ( (pos_l_raw-this->pos_l_history_) < -0.8*this->encoder_counts_pre_round_ )
        {
            this->wheel_l_round_num_ += 1;
        }
        else if ( (pos_l_raw-this->pos_l_history_) > 0.8*this->encoder_counts_pre_round_ )
        {
            this->wheel_l_round_num_ -= 1;
        }


        if ( (pos_r_raw-this->pos_r_history_) < -0.8*this->encoder_counts_pre_round_ )
        {
            this->wheel_r_round_num_ += 1;
        }
        else if ( (pos_r_raw-this->pos_r_history_) > 0.8*this->encoder_counts_pre_round_ )
        {
            this->wheel_r_round_num_ -= 1;
        }

        pos_l = ((double)pos_l_raw + (double)this->wheel_l_round_num_*this->encoder_counts_pre_round_) / this->encoder_counts_pre_round_ *2*PI;
        pos_r = ((double)pos_r_raw + (double)this->wheel_r_round_num_*this->encoder_counts_pre_round_) / this->encoder_counts_pre_round_ *2*PI;

        this->pos_l_history_ = pos_l_raw;
        this->pos_r_history_ = pos_r_raw;

    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The Read() call timed out waiting for additional data." << std::endl ;
    }


}
