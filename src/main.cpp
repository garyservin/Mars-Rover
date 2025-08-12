/*
 * Code for controlling my Rover. Controlled using generic 6 channel RC Transmitter, Websocket or Lora.
 *
 * On the transmitter I use two joysticks for channels 1-4 for, and two switches for channel 5-6.
 * Joysticks are used both for steering the Rover and for moving each axis of the arm.
 * When switch1 is:
 *   -LOW: joysticks are used for driving the rover
 *   -MID: Wheels will be set correct angles for the rover to be able to rotate on the spot
 *         by running each side of wheels in opposite direction. The direction is descided by the steer joystick.
 *   -HIGH: Arm mode, the two joysticks are used for moving axis 1-4 on the arm. Switch2 is used to switch beween moving
 *          axis 1-4 and axis 5-6 (the gripper). This will be changed in the future, steering each axis manually is not really a good way, inverse kinematics is TODO.
 */

#include <Arduino.h>
#include "rover_config.h"
#include <ESP32Servo.h>
#include "Wire.h"
#include "rc_receiver_rmt.h"
#include "switch_checker.h"
#include "arm.h"
#include "rover_servo.h"
#include "rover_head.h"
#include "rover_driving.h"

#include <SPI.h>

#define DEBUG

#define ACCEL_READ_PERIOD_MS 300
#define PING_WS_ITERATIONS (1000 / ACCEL_READ_PERIOD_MS)

#ifdef DEBUG
#define LOG Serial.print
#define LOGF Serial.printf
#define LOGLN Serial.println
#else
#define LOG(msg)
#define LOGF(...)
#define LOGLN(msg)
#endif

static void handle_robot_arm_servo(ArmAxis arm_axis, uint16_t channel);
static uint16_t get_controller_channel_value(uint8_t channel);
static uint16_t filter_signal(uint16_t *signals);
static void handle_arm_mode_changed(ArmMode mode);
static void handle_controller_disconnected(uint16_t last_sampled_signal);
static void handle_rover_mode_changed(RoverMode mode);

static const char *TAG = "ROVER_MAIN";

static uint16_t rc_values[RC_NUM_CHANNELS][RC_FILTER_SAMPLES];
static uint8_t rc_value_index = 0;

static RoverMode current_rover_mode = DRIVE_TURN_NORMAL;
static ArmMode current_arm_mode = ARM_MODE_MOVE;

void setup()
{
  Serial.begin(SERIAL_PORT_SPEED);
  Wire.begin(ROVER_SDA_PIN, ROVER_SCL_PIN);
  Wire.setClock(400000);
  xSemaphoreHandle i2cSemaphoreHandle = xSemaphoreCreateBinary();
  assert(i2cSemaphoreHandle != NULL);
  xSemaphoreGive(i2cSemaphoreHandle);

  rover_servo_init(i2cSemaphoreHandle);
  handle_controller_disconnected(0);

  rc_receiver_rmt_init();

  init_switch_checker(RC_LOW, RC_ROVER_MODE_ROVER_CHANNEL, RC_ARM_MODE_ROVER_CHANNEL, &handle_rover_mode_changed, &handle_arm_mode_changed, false);

  rover_driving_init();
  // arm_init();
  // rover_head_init();

  LOGF("Rover Ready! Core: %d", xPortGetCoreID());

  pinMode(2, OUTPUT);
}

void loop()
{
  digitalWrite(2, !digitalRead(2));

  switch (current_rover_mode)
  {
  case DRIVE_TURN_NORMAL:
  case DRIVE_TURN_SPIN:
  {
    uint16_t motor_signal = RC_CENTER;

    rc_values[RC_STEER_CHANNEL][rc_value_index] = get_controller_channel_value(RC_STEER_CHANNEL);
    rc_values[RC_MOTOR_CHANNEL][rc_value_index] = get_controller_channel_value(RC_MOTOR_CHANNEL);
    // rc_values[RC_HEAD_PITCH_CHANNEL][rc_value_index] = get_controller_channel_value(RC_HEAD_PITCH_CHANNEL);
    // rc_values[RC_HEAD_YAW_CHANNEL][rc_value_index] = get_controller_channel_value(RC_HEAD_YAW_CHANNEL);

    if (current_rover_mode == DRIVE_TURN_NORMAL)
    {
      motor_signal = filter_signal(rc_values[RC_MOTOR_CHANNEL]);
    }
    else if (current_rover_mode == DRIVE_TURN_SPIN)
    {
      motor_signal = filter_signal(rc_values[RC_STEER_CHANNEL]);
    }

    rover_driving_move(motor_signal);
    rover_driving_steer(filter_signal(rc_values[RC_STEER_CHANNEL]));

    // uint16_t yawUs = filter_signal(rc_values[RC_HEAD_YAW_CHANNEL]);
    // uint16_t pitchUs = filter_signal(rc_values[RC_HEAD_PITCH_CHANNEL]);

    // rover_head_yaw(yawUs);
    // rover_head_pitch(pitchUs);
    break;
  }
    // case ROBOT_ARM:
    //   if (current_arm_mode == ARM_MODE_MOVE)
    //   {
    //     rc_values[RC_SERVO1_CHANNEL][rc_value_index] = get_controller_channel_value(RC_SERVO1_CHANNEL);
    //     rc_values[RC_SERVO2_CHANNEL][rc_value_index] = get_controller_channel_value(RC_SERVO2_CHANNEL);
    //     rc_values[RC_SERVO3_CHANNEL][rc_value_index] = get_controller_channel_value(RC_SERVO3_CHANNEL);
    //     rc_values[RC_SERVO4_CHANNEL][rc_value_index] = get_controller_channel_value(RC_SERVO4_CHANNEL);

    //    handle_robot_arm_servo(ARM_AXIS_1, RC_SERVO1_CHANNEL);
    //    handle_robot_arm_servo(ARM_AXIS_2, RC_SERVO2_CHANNEL);
    //    handle_robot_arm_servo(ARM_AXIS_3, RC_SERVO3_CHANNEL);
    //    handle_robot_arm_servo(ARM_AXIS_4, RC_SERVO4_CHANNEL);
    //  }
    //  else if (current_arm_mode == ARM_MODE_GRIPPER)
    //  {
    //    rc_values[RC_SERVO5_CHANNEL][rc_value_index] = get_controller_channel_value(RC_SERVO5_CHANNEL);
    //    rc_values[RC_SERVO6_CHANNEL][rc_value_index] = get_controller_channel_value(RC_SERVO6_CHANNEL);

    //    handle_robot_arm_servo(ARM_AXIS_5, RC_SERVO5_CHANNEL);
    //    handle_robot_arm_servo(ARM_AXIS_6, RC_SERVO6_CHANNEL);
    //  }
    //  break;
  }
  rc_value_index = (rc_value_index + 1) % RC_FILTER_SAMPLES;

  vTaskDelay(pdMS_TO_TICKS(20));
}

static void handle_robot_arm_servo(ArmAxis arm_axis, uint16_t channel)
{

  uint16_t speed;
  uint16_t signal = filter_signal(rc_values[channel]);
  if (signal < RC_CENTER && signal > 0)
  {
    signal = 2 * RC_CENTER - signal;
    speed = map(signal, RC_CENTER, RC_HIGH, ARM_MIN_SPEED, ARM_MAX_SPEED);
    arm_move_axis_us(arm_axis, RC_LOW, speed);
  }
  else if (signal > RC_CENTER)
  {
    speed = map(signal, RC_CENTER, RC_HIGH, ARM_MIN_SPEED, ARM_MAX_SPEED);
    arm_move_axis_us(arm_axis, RC_HIGH, speed);
  }
  else
  {
    arm_pause(arm_axis);
  }
}

static uint16_t get_controller_channel_value(uint8_t channel)
{
  uint16_t channel_value = 0;

  channel_value = rc_receiver_rmt_get_val(channel);

  if (channel_value == 0)
  {
    channel_value = RC_CENTER;
  }

  return channel_value;
}

static uint16_t filter_signal(uint16_t *signals)
{
  uint32_t signal = 0;
  for (uint8_t i = 0; i < RC_FILTER_SAMPLES; i++)
  {
    signal += signals[i];
  }
  signal = signal / RC_FILTER_SAMPLES;

  if (signal <= 800)
    return RC_CENTER; // If we are unlucky in timing when RC controller disconnect we might get some random low signal
  if (signal > 800 && signal < 1010)
    return RC_LOW;
  if (signal > 1990)
    return RC_HIGH;
  if (signal > 1480 && signal < 1520)
    return RC_CENTER;
  return signal;
  ;
}

static void handle_rover_mode_changed(RoverMode mode)
{
  LOGF("ROVER MODE CHANGED: %d\n", mode);
  current_rover_mode = mode;
  rover_driving_set_drive_mode(mode);
}

static void handle_arm_mode_changed(ArmMode mode)
{
  LOGF("ARM MODE CHANGED: %d\n", mode);
  current_arm_mode = mode;
}

static void handle_controller_disconnected(uint16_t last_sampled_signal)
{
  // Set all RC values to mid position if signal is 0 (controller disconnected)
  if (last_sampled_signal == 0)
  {
    for (uint8_t channel = 0; channel < RC_NUM_CHANNELS; channel++)
    {
      for (uint8_t sample = 0; sample < RC_FILTER_SAMPLES; sample++)
      {
        rc_values[channel][sample] = RC_CENTER;
      }
    }
  }
}