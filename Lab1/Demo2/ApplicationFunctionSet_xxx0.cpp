/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-28 14:10:45
 * @LastEditors: Changhua
 * @Description: SmartRobot robot tank
 * @FilePath: 
 */
#include <HardwareSerial.h>
#include <stdio.h>
#include <string.h>
#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"
#include "ArduinoJson-v6.11.1.h" // ArduinoJson
#include "MPU6050_getdata.h"

#define _is_print 1
#define _Test_print 0

ApplicationFunctionSet Application_FunctionSet;

/* Hardware device member objects */
MPU6050_getdata AppMPU6050getdata;
DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_ULTRASONIC AppULTRASONIC;
DeviceDriverSet_Servo AppServo;

/* Function to check range */
static boolean function_xxx(long x, long s, long e) {
  if (s <= x && x <= e)
    return true;
  else
    return false;
}

/* Motion direction control sequence */
enum SmartRobotCarMotionControl {
  Forward,       //(1)
  Backward,      //(2)
  Left,          //(3)
  Right,         //(4)
  LeftForward,   //(5)
  LeftBackward,  //(6)
  RightForward,  //(7)
  RightBackward, //(8)
  stop_it        //(9)
};               

/* Mode control sequence */
enum SmartRobotCarFunctionalModel {
  Standby_mode,           /* Idle mode */
  TraceBased_mode,        /* Tracking mode */
  ObstacleAvoidance_mode, /* Obstacle avoidance mode */
  Follow_mode,            /* Follow mode */
  Rocker_mode,            /* Rocker mode */
  SurfaceAvoidance_mode,  /* Surface avoidance mode */
};

/* Control management members */
struct Application_xxx {
  SmartRobotCarMotionControl Motion_Control;
  SmartRobotCarFunctionalModel Functional_Mode;
  unsigned long CMD_CarControl_Millis;
  unsigned long CMD_LightingControl_Millis;
};
Application_xxx Application_SmartRobotCarxxx0;

bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void);
static void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit);
static void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed);

void ApplicationFunctionSet::ApplicationFunctionSet_Init(void) {
  bool res_error = true;
  Serial.begin(9600);
  AppMotor.DeviceDriverSet_Motor_Init();
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Init();
  res_error = AppMPU6050getdata.MPU6050_dveInit();
  AppMPU6050getdata.MPU6050_calibration();

  while (Serial.read() >= 0) {
    /* Clear the serial buffer */
  }
  delay(2000);
  Application_SmartRobotCarxxx0.Functional_Mode = SurfaceAvoidance_mode;
}

/*
 * Linear motion control:
 * direction: Choose direction forward/backward
 * directionRecord: Direction record (updated when first entering the function, controls yaw)
 * speed: Input speed (0-255)
 * Kp: Proportional constant for position error amplification
 * UpperLimit: Upper limit of maximum output control amount
 */
static void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit) {
  static float Yaw; 
  static float yaw_So = 0;
  static uint8_t en = 110;
  static unsigned long is_time;

  if (en != directionRecord || millis() - is_time > 10) {
    AppMotor.DeviceDriverSet_Motor_control(direction_void, 0, direction_void, 0, control_enable); // Motor control
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
    is_time = millis();
  }

  if (en != directionRecord || Application_FunctionSet.Car_LeaveTheGround == false) {
    en = directionRecord;
    yaw_So = Yaw;
  }

  int R = (Yaw - yaw_So) * Kp + speed;
  R = constrain(R, 10, UpperLimit);
  int L = (yaw_So - Yaw) * Kp + speed;
  L = constrain(L, 10, UpperLimit);

  if (direction == Forward) {
    AppMotor.DeviceDriverSet_Motor_control(direction_just, R, direction_just, L, control_enable);
  } else if (direction == Backward) {
    AppMotor.DeviceDriverSet_Motor_control(direction_back, L, direction_back, R, control_enable);
  }
}

/*
 * Motion control:
 * 1# direction: forward, backward, left, right, etc.
 * 2# speed: speed (0-255)
 */
static void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed) {
  static uint8_t directionRecord = 0;
  uint8_t Kp = 2, UpperLimit = 180;
  uint8_t speed = is_speed;

  switch (direction) {
    case Forward:
      if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode) {
        AppMotor.DeviceDriverSet_Motor_control(direction_just, speed, direction_just, speed, control_enable);
      } else {
        ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Forward, directionRecord, speed, Kp, UpperLimit);
        directionRecord = 1;
      }
      break;
    case Backward:
      if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode) {
        AppMotor.DeviceDriverSet_Motor_control(direction_back, speed, direction_back, speed, control_enable);
      } else {
        ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Backward, directionRecord, speed, Kp, UpperLimit);
        directionRecord = 2;
      }
      break;
    case Left:
      directionRecord = 3;
      AppMotor.DeviceDriverSet_Motor_control(direction_just, speed, direction_back, speed, control_enable);
      break;
    case Right:
      directionRecord = 4;
      AppMotor.DeviceDriverSet_Motor_control(direction_back, speed, direction_just, speed, control_enable);
      break;
    case LeftForward:
      directionRecord = 5;
      AppMotor.DeviceDriverSet_Motor_control(direction_just, speed, direction_just, speed / 2, control_enable);
      break;
    case LeftBackward:
      directionRecord = 6;
      AppMotor.DeviceDriverSet_Motor_control(direction_back, speed, direction_back, speed / 2, control_enable);
      break;
    case RightForward:
      directionRecord = 7;
      AppMotor.DeviceDriverSet_Motor_control(direction_just, speed / 2, direction_just, speed, control_enable);
      break;
    case RightBackward:
      directionRecord = 8;
      AppMotor.DeviceDriverSet_Motor_control(direction_back, speed / 2, direction_back, speed, control_enable);
      break;
    case stop_it:
      directionRecord = 9;
      AppMotor.DeviceDriverSet_Motor_control(direction_void, 0, direction_void, 0, control_enable);
      break;
    default:
      directionRecord = 10;
      break;
  }
}

static void delay_xxx(uint16_t _ms) {
  for (unsigned long i = 0; i < _ms; i++) {
    delay(1);
  }
}

/*
 * Obstacle avoidance function
 */
void ApplicationFunctionSet::ApplicationFunctionSet_Obstacle(void) {
  static boolean first_is = true;
  if (Application_SmartRobotCarxxx0.Functional_Mode == ObstacleAvoidance_mode) {
    uint8_t switc_ctrl = 0;
    uint16_t get_Distance;
    if (Car_LeaveTheGround == false) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      return;
    }
    if (first_is == true) {
      AppServo.DeviceDriverSet_Servo_control(90);
      first_is = false;
    }

    AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&get_Distance);
    Serial.println(get_Distance);
    if (function_xxx(get_Distance, 0, 20)) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

      for (int i = 1; i < 6; i += 2) {
        AppServo.DeviceDriverSet_Servo_control(30 * i);
        delay_xxx(1);
        AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&get_Distance);

        if (function_xxx(get_Distance, 0, 20)) {
          ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
          if (5 == i) {
            ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, 150);
            delay_xxx(500);
            ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 150);
            delay_xxx(50);
            first_is = true;
            break;
          }
        } else {
          switc_ctrl = 0;
          switch (i) {
            case 1:
              ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 150);
              break;
            case 3:
              ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 150);
              break;
            case 5:
              ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 150);
              break;
          }
          delay_xxx(50);
          first_is = true;
          break;
        }
      }
    } else {
      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 150);
    }
  } else {
    first_is = true;
  }
}

/*
 * Updated Surface Avoidance Function with Diagnostics
 */
void ApplicationFunctionSet::ApplicationFunctionSet_SurfaceAvoidance(void) {
    static boolean first_is = true;
    static boolean was_in_mode = false; // Tracks whether the robot was previously in Surface Avoidance mode
    static uint16_t initial_Distance = 0; // Stores the initial distance to the surface
    uint16_t get_Distance;

    // Check if the current mode is Surface Avoidance
    if (Application_SmartRobotCarxxx0.Functional_Mode == SurfaceAvoidance_mode) {
        // Print only once when entering the mode
        if (!was_in_mode) {
            Serial.println("Surface Avoidance Mode Activated");
            was_in_mode = true;
        }

        // Check if the car is on the ground
        if (!Car_LeaveTheGround) {
            Serial.println("Car is off the ground. Stopping...");
            ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
            return;
        }

        // Store the initial surface reading when entering the mode for the first time
        if (first_is) {
            AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&initial_Distance);
            Serial.print("Initial Surface Distance Set: ");
            Serial.println(initial_Distance);
            first_is = false;
        }

        // Get the current distance reading from the ultrasonic sensor
        AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&get_Distance);
        Serial.print("Current Distance: ");
        Serial.println(get_Distance);

        // Check if sensor values are reasonable
        if (get_Distance == 0 || initial_Distance == 0) {
            Serial.println("Error: Sensor readings are zero, check sensor connections.");
            return;
        }

        // Log the difference between initial and current distance
        int distanceDifference = get_Distance - initial_Distance;
        Serial.print("Distance Difference: ");
        Serial.println(distanceDifference);

        // Check if the robot is no longer on the surface (detects a significant drop)
        if (get_Distance > initial_Distance + 20) { // Adjust the threshold value as needed
            Serial.println("Surface Lost! Stopping Robot...");
            ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

            // Redirect the robot back to the surface (e.g., move back and turn)
            Serial.println("Reversing to find surface...");
            ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, 50);
            delay_xxx(500);

            Serial.println("Turning right to reposition...");

            int speedOfTurn = random(100, 256);
            ApplicationFunctionSet_SmartRobotCarMotionControl(Right, speedOfTurn);
            delay_xxx(500);

            // Resetting the first-time flag to recalibrate surface distance after re-alignment
            first_is = true;
            Serial.println("Recalibrating surface distance...");
        } else {
            // Keep moving forward if still on the surface
            Serial.println("Surface is stable. Moving forward...");
            ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 50);
        }
    } else {
        // Print the exit message only once when leaving the mode
        if (was_in_mode) {
            Serial.println("Exiting Surface Avoidance Mode.");
            was_in_mode = false;
            first_is = true; // Reset the flag when exiting the mode
        }
    }
}
