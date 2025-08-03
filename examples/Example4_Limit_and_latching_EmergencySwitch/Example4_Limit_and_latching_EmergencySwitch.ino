#include <ESP_FlexyStepper.h>

// asignación de pines IO
const int MOTOR_STEP_PIN = 33;
const int MOTOR_DIRECTION_PIN = 25;
const int EMERGENCY_STOP_PIN = 13; // botón de emergencia
const int LIMIT_SWITCH_PIN = 32;   // finales de carrera en serie

// parámetros de movimiento
const int DISTANCE_TO_TRAVEL_IN_STEPS = 2000;
const int SPEED_IN_STEPS_PER_SECOND = 300;
const int ACCELERATION_IN_STEPS_PER_SECOND = 800;
const int DECELERATION_IN_STEPS_PER_SECOND = 800;

ESP_FlexyStepper stepper;
int previousDirection = 1;

// rebote de finales de carrera
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 100;
byte limitSwitchState = 0;
byte oldConfirmedLimitSwitchState = 0;

QueueHandle_t emergencyQueue;

// ISR del botón de emergencia
void IRAM_ATTR emergencySwitchHandler()
{
  bool pressed = (digitalRead(EMERGENCY_STOP_PIN) == LOW);
  BaseType_t woken = pdFALSE;
  xQueueSendFromISR(emergencyQueue, &pressed, &woken);
  if (woken)
    portYIELD_FROM_ISR();
}

// tarea que gestiona la parada de emergencia
void emergencyTask(void *p)
{
  bool pressed;
  for (;;)
  {
    if (xQueueReceive(emergencyQueue, &pressed, portMAX_DELAY) == pdTRUE)
    {
      if (pressed)
        stepper.emergencyStop(true);
      else
        stepper.releaseEmergencyStop();
    }
  }
}

void limitSwitchHandler()
{
  limitSwitchState = digitalRead(LIMIT_SWITCH_PIN);
  lastDebounceTime = millis();
}

void setup()
{
  Serial.begin(115200);
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  emergencyQueue = xQueueCreate(5, sizeof(bool));
  xTaskCreate(emergencyTask, "emerg", 2048, NULL, configMAX_PRIORITIES - 1, NULL);
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_STOP_PIN), emergencySwitchHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), limitSwitchHandler, CHANGE);
  stepper.setDirectionToHome(-1);
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
  stepper.setSpeedInStepsPerSecond(SPEED_IN_STEPS_PER_SECOND);
  stepper.setAccelerationInStepsPerSecondPerSecond(ACCELERATION_IN_STEPS_PER_SECOND);
  stepper.setDecelerationInStepsPerSecondPerSecond(DECELERATION_IN_STEPS_PER_SECOND);
}

void loop()
{
  if (limitSwitchState != oldConfirmedLimitSwitchState && (millis() - lastDebounceTime) > debounceDelay)
  {
    oldConfirmedLimitSwitchState = limitSwitchState;
    Serial.printf("Cambio en final de carrera. Nuevo estado %i\n", limitSwitchState);
    if (limitSwitchState == HIGH)
    {
      stepper.setLimitSwitchActive(stepper.LIMIT_SWITCH_COMBINED_BEGIN_AND_END);
    }
    else
    {
      stepper.clearLimitSwitchActive();
    }
  }

  if (stepper.getDirectionOfMotion() == 0)
  {
    previousDirection *= -1;
    long target = DISTANCE_TO_TRAVEL_IN_STEPS * previousDirection;
    stepper.setTargetPositionRelativeInSteps(target);
  }

  stepper.processMovement();
}

