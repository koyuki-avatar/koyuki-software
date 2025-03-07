/*
MIT License

Copyright (c) 2022 Masaaki Hijikata

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
/**
* @file blvr_comunicator.h
* @brief header file for blvr motor comunication
* @author Masaaki Hijikata <hijikata@ir.utsunomiya-u.ac.jp>, Utsunomiya Univ.
* @date 20210218
* @details  
*/
extern "C" {
#include <time.h>
#include <termios.h>
}
#include <stdint.h>
#include <string>

#ifndef __BLVR_COMUNICATOR_HPP__
#define __BLVR_COMUNICATOR_HPP__


class BlvrComunicator
{
public:
  static constexpr char BLVRD_DEFAULT_DEVICE[] = "/dev/ttyUSB0";
  static const int BLVRD_DEFAULT_FREQUENCY = 10;

  static const int BAUDRATE = B115200;
  static const int BLVR_MAX_RPM = 4000;
  static constexpr double BLVR_RATED_TORQUE = 0.64; /*Nm*/
  static const int BLVR_STEP_TO_ONESHOT = 36000;

  static const int ACCESS_DELAY = 3500; /* us */
  static const int READ_TIMEOUT = 100*2;

  static const int MSEC = 1000000; //nanosec to msec
  static const int USEC = 1000;    //nanosec to usec

  static constexpr struct timespec BROADCAST_DELAY    = {0,  10 * MSEC};
  static constexpr struct timespec RESPONSE_DELAY     = {0,   5 * MSEC};
  static constexpr struct timespec READ_RETRY_DELAY   = {0,    5 * MSEC};
  static constexpr struct timespec CHARACTER_DELAY    = {0,   10 * USEC};

  static const int MESSAGE_BUF_SIZE = 255;

  enum class return_type : uint8_t
  {
    SUCCESS = 0,
    ERROR = 1
  };
  enum Mode
  {
    NOCONTROL,
    ABSOLUTE_POSITION,
    RELATIVE_POSITION_FROM_TARGET,
    RELATIVE_POSITION_FROM_CURRENT,
    RELATIVE_POSITIONING_SPEEDS_CONTROL_FROM_TARGET = 5,
    RELATIVE_POSITIONING_SPEEDS_CONTROL_FROM_CURRENT,
    CONTINUOUS_OPERATION_BY_RPM = 16,
    CONTINUOUS_OPERATION_BY_PUSH,
    CONTINUOUS_OPERATION_BY_TORQUE,
    PUSHING_OPERATION_FROM_ABSOLUTE_POSITION = 20,
    PUSHING_OPERATION_FROM_TARGET_POSITION,
    PUSHING_OPERATION_FROM_CURRENT_POSITION,
    MOTION_CONTINUOUS_OPERATION_BY_RPM = 48,
    MOTION_CONTINUOUS_OPERATION_BY_PUSH,
    MOTION_CONTINUOUS_OPERATION_BY_TORQUE,
  };

  BlvrComunicator()
  : is_open(false)
  , blvr_port(0) {};
  ~BlvrComunicator() {};

  return_type openDevice(std::string& device);
  void        closeDevice();

  int   directDataDrive(int ch, Mode mode, int position, int rpm, int acc_rate, int dec_rate, int torque);
  int   setExcitation(int ch);

  int   readStep(int ch, int *step);
  int   readRpm(int ch, int *rpm);
  int   readAlarm(int ch, int *alarm);
  int   readWarning(int ch, int *warning);
  int   readTorque(int ch, int *torque);
  int   resetAlarm(int ch);

    int comm_err_cnt;

  bool is_open;

private:
  uint16_t makeCrc16(uint8_t *p, size_t len);

  int blvr_port;
};

#endif /* __BLVR_COMUNICATOR_HPP__ */
