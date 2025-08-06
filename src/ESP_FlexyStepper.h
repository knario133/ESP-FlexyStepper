
//      ******************************************************************
//      *                                                                *
//      *            Archivo de cabecera para ESP-FlexyStepper            *
//      *                                                                *
//      *            Paul Kerspe                     4.6.2020            *
//      *      basado en el concepto de FlexyStepper por Stan Reifel     *
//      *                                                                *
//      ******************************************************************

// MIT License
//
// Copyright (c) 2020 Paul Kerspe
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is furnished
// to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// Esta biblioteca se basa en los trabajos de Stan Reifel en su biblioteca FlexyStepper:
// https://github.com/Stan-Reifel/FlexyStepper

#ifndef ESP_FlexyStepper_h
#define ESP_FlexyStepper_h

#ifdef ESP32
//
// #elif defined(ESP8266)
//
#else
#error Platform not supported, only ESP32 modules are currently supported
#endif

#include <Arduino.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <driver/gpio.h>

typedef void (*callbackFunction)(void);
typedef void (*positionCallbackFunction)(long);

class ESP_FlexyStepper
{
public:
  ESP_FlexyStepper();
  ~ESP_FlexyStepper();
  // funciones de servicio
  bool startAsService(int coreNumber = 1);
  void stopService(void);
  bool isStartedAsService(void);

  // funciones de configuración de IO y ayuda / depuración
  void connectToPins(byte stepPinNumber, byte directionPinNumber = 255, bool useOpenDrain = false);
  void setBrakePin(signed char brakePin, byte activeState = ESP_FlexyStepper::ACTIVE_HIGH);
  void setEnablePin(signed char enablePin, byte activeState = ESP_FlexyStepper::ACTIVE_LOW);
  long getTaskStackHighWaterMark(void);
  void clearLimitSwitchActive(void);
  bool motionComplete();
  int getDirectionOfMotion(void);
  bool isMovingTowardsHome(void);
  void emergencyStop(bool holdUntilReleased = false);
  void releaseEmergencyStop(void);
  void activateBrake(void);
  void deactivateBrake(void);
  bool isBrakeActive(void);
  void enableDriver(void);
  void disableDriver(void);
  bool isDriverEnabled(void);
  // función central para calcular la señal del siguiente paso
  bool processMovement(void);

  // registro de funciones de callback
  void registerHomeReachedCallback(callbackFunction homeReachedCallbackFunction);
  void registerLimitReachedCallback(callbackFunction limitSwitchTriggerdCallbackFunction);
  void registerTargetPositionReachedCallback(positionCallbackFunction targetPositionReachedCallbackFunction);
  void registerEmergencyStopTriggeredCallback(callbackFunction emergencyStopTriggerdCallbackFunction);
  void registerEmergencyStopReleasedCallback(callbackFunction emergencyStopReleasedCallbackFunction);

  // funciones de configuración
  void setStepsPerMillimeter(float motorStepPerMillimeter);
  void setStepsPerRevolution(float motorStepPerRevolution);
  void setSpeedInStepsPerSecond(float speedInStepsPerSecond);
  void setSpeedInMillimetersPerSecond(float speedInMillimetersPerSecond);
  void setSpeedInRevolutionsPerSecond(float speedInRevolutionsPerSecond);
  void setAccelerationInMillimetersPerSecondPerSecond(float accelerationInMillimetersPerSecondPerSecond);
  void setAccelerationInRevolutionsPerSecondPerSecond(float accelerationInRevolutionsPerSecondPerSecond);
  void setDecelerationInMillimetersPerSecondPerSecond(float decelerationInMillimetersPerSecondPerSecond);
  void setDecelerationInRevolutionsPerSecondPerSecond(float decelerationInRevolutionsPerSecondPerSecond);
  void setAccelerationInStepsPerSecondPerSecond(float accelerationInStepsPerSecondPerSecond);
  void setDecelerationInStepsPerSecondPerSecond(float decelerationInStepsPerSecondPerSecond);
  void setDirectionToHome(signed char directionTowardHome);
  void setLimitSwitchActive(signed char limitSwitchType);
  void setContinuousVelocityInStepsPerSecond(float velocity);
  void setAutoDisable(bool enable, unsigned long delayMs);

  void setBrakeEngageDelayMs(unsigned long);
  void setBrakeReleaseDelayMs(signed long);

  float getCurrentVelocityInStepsPerSecond();
  float getCurrentVelocityInRevolutionsPerSecond();
  float getCurrentVelocityInMillimetersPerSecond(void);

  float getConfiguredAccelerationInStepsPerSecondPerSecond();
  float getConfiguredAccelerationInRevolutionsPerSecondPerSecond();
  float getConfiguredAccelerationInMillimetersPerSecondPerSecond();

  float getConfiguredDecelerationInStepsPerSecondPerSecond();
  float getConfiguredDecelerationInRevolutionsPerSecondPerSecond();
  float getConfiguredDecelerationInMillimetersPerSecondPerSecond();

  // funciones de posicionamiento
  void setCurrentPositionInSteps(long currentPositionInSteps);
  void setCurrentPositionInMillimeters(float currentPositionInMillimeters);
  void setCurrentPositionInRevolutions(float currentPositionInRevolutions);

  long getCurrentPositionInSteps();
  float getCurrentPositionInRevolutions();
  float getCurrentPositionInMillimeters();

  void startJogging(signed char direction);
  void stopJogging();
  void goToLimitAndSetAsHome(callbackFunction callbackFunctionForHome = NULL, long maxDistanceToMoveInSteps = 2000000000L);
  void goToLimit(signed char direction, callbackFunction callbackFunctionForLimit = NULL);

  void setCurrentPositionAsHomeAndStop(void);
  void setTargetPositionToStop();
  long getDistanceToTargetSigned(void);

  void setTargetPositionInSteps(long absolutePositionToMoveToInSteps);
  void setTargetPositionInMillimeters(float absolutePositionToMoveToInMillimeters);
  void setTargetPositionInRevolutions(float absolutePositionToMoveToInRevolutions);
  void setTargetPositionRelativeInSteps(long distanceToMoveInSteps);
  void setTargetPositionRelativeInMillimeters(float distanceToMoveInMillimeters);
  void setTargetPositionRelativeInRevolutions(float distanceToMoveInRevolutions);

  long getTargetPositionInSteps();
  float getTargetPositionInMillimeters();
  float getTargetPositionInRevolutions();

  // funciones bloqueantes
  void moveToPositionInSteps(long absolutePositionToMoveToInSteps);
  void moveToPositionInMillimeters(float absolutePositionToMoveToInMillimeters);
  void moveToPositionInRevolutions(float absolutePositionToMoveToInRevolutions);
  void moveRelativeInSteps(long distanceToMoveInSteps);
  void moveRelativeInMillimeters(float distanceToMoveInMillimeters);
  void moveRelativeInRevolutions(float distanceToMoveInRevolutions);

  bool moveToHomeInSteps(signed char directionTowardHome, float speedInStepsPerSecond, long maxDistanceToMoveInSteps, int homeSwitchPin, callbackFunction homeReachedCallback = NULL);
  bool moveToHomeInMillimeters(signed char directionTowardHome, float speedInMillimetersPerSecond, long maxDistanceToMoveInMillimeters, int homeLimitSwitchPin, callbackFunction homeReachedCallback = NULL);
  bool moveToHomeInRevolutions(signed char directionTowardHome, float speedInRevolutionsPerSecond, long maxDistanceToMoveInRevolutions, int homeLimitSwitchPin, callbackFunction homeReachedCallback = NULL);

  static const signed char LIMIT_SWITCH_BEGIN = -1;  // interruptor al inicio
  static const signed char LIMIT_SWITCH_END = 1;     // interruptor al final
  static const signed char LIMIT_SWITCH_COMBINED_BEGIN_AND_END = 2;
  static const byte ACTIVE_HIGH = 1;
  static const byte ACTIVE_LOW = 2;

private:
  callbackFunction _homeReachedCallback = NULL;
  callbackFunction _limitTriggeredCallback = NULL;
  callbackFunction _emergencyStopTriggeredCallback = NULL;
  callbackFunction _emergencyStopReleasedCallback = NULL;
  positionCallbackFunction _targetPositionReachedCallback = NULL;
  callbackFunction _callbackFunctionForGoToLimit = NULL;

  static void taskRunner(void *parameter);

  void DeterminePeriodOfNextStep();
  void triggerBrakeIfNeededOrSetTimeout(void);

  byte stepPin;
  signed char brakePin = -1;
  signed char enablePin = -1;
  byte brakePinActiveState = ACTIVE_HIGH;
  byte enablePinActiveState = ACTIVE_HIGH;
  unsigned long _brakeEngageDelayMs = 0;
  signed long _brakeReleaseDelayMs = -1;
  unsigned long _timeToEngangeBrake = LONG_MAX;
  unsigned long _timeToReleaseBrake = LONG_MAX;
  bool _isBrakeConfigured = false;
  bool _isEnableConfigured = false;
  bool _hasMovementOccuredSinceLastBrakeRelease = true;

  byte directionPin;
  bool _isBrakeActive = false;
  bool _isDriverEnabled = false;
  float stepsPerMillimeter;
  float stepsPerRevolution;
  int directionOfMotion;
  long currentPosition_InSteps;
  long targetPosition_InSteps;
  float desiredSpeed_InStepsPerSecond;
  float desiredPeriod_InUSPerStep;
  float acceleration_InStepsPerSecondPerSecond;
  float acceleration_InStepsPerUSPerUS;
  float deceleration_InStepsPerSecondPerSecond;
  float deceleration_InStepsPerUSPerUS;
  float periodOfSlowestStep_InUS;
  float minimumPeriodForAStoppedMotion;
  float nextStepPeriod_InUS;
  unsigned long lastStepTime_InUS;
  float currentStepPeriod_InUS;
  bool emergencyStopActive;
  bool holdEmergencyStopUntilExplicitRelease;
  signed char directionTowardsHome;
  signed char lastStepDirectionBeforeLimitSwitchTrigger;
  // verdadero si la posición actual es igual a la posición de "home"
  bool isCurrentlyHomed;
  bool isOnWayToHome = false;
  bool isOnWayToLimit = false;
  bool firstProcessingAfterTargetReached = false;
  // Tipo de interruptor de límite activo
  signed char activeLimitSwitch;
  bool limitSwitchCheckPeformed;
  // 0 si el motor puede moverse en ambas direcciones, en otro caso indica la dirección prohibida
  signed char disallowedDirection;

  // estado para el proceso de homing no bloqueante
  enum HomingState : uint8_t
  {
    HOMING_IDLE = 0,
    HOMING_MOVE_TO_SWITCH,
    HOMING_WAIT_BEFORE_BACKOFF,
    HOMING_MOVE_OFF_SWITCH,
    HOMING_WAIT_BEFORE_FINAL,
    HOMING_FINAL_APPROACH
  };
  HomingState _homingState = HOMING_IDLE;
  unsigned long _homingStateStartTime = 0;
  float _homingOriginalSpeed = 0;
  float _homingSearchSpeed = 0;
  long _homingMaxDistance = 0;
  int _homingSwitchPin = -1;
  signed char _homingDirection = 0;

  // soporte para movimiento continuo
  bool _continuousMovement = false;

  // auto desenergizar
  bool _autoDisable = false;
  unsigned long _autoDisableDelay = 0;
  unsigned long _lastActiveTime = 0;

  TaskHandle_t xHandle = NULL;
};

// ------------------------------------ End ---------------------------------
#endif
