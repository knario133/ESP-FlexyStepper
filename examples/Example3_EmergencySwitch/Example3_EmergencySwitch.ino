
//      ******************************************************************
//      *                                                                *
//      *   Uso de un botón de emergencia para detener el movimiento     *
//      *                                                                *
//      *            Paul Kerspe               4.6.2020                  *
//      *                                                                *
//      ******************************************************************

// Este ejemplo muestra cómo utilizar un interruptor de emergencia con una
// interrupción para detener el movimiento inmediatamente.
//
// Cambia los números de pines para que coincidan con tu configuración
// y ajusta velocidad y distancia según sea necesario.
//
// Documentación de la librería:
//    https://github.com/pkerspe/ESP-FlexyStepper/blob/master/README.md
//

#include <ESP_FlexyStepper.h>

// asignación de pines IO
const int MOTOR_STEP_PIN = 33;
const int MOTOR_DIRECTION_PIN = 25;
const int EMERGENCY_STOP_PIN = 13; // pin donde está conectado el botón de parada

// configuración de velocidad
const int DISTANCE_TO_TRAVEL_IN_STEPS = 500;
const int SPEED_IN_STEPS_PER_SECOND = 800;
const int ACCELERATION_IN_STEPS_PER_SECOND = 500;
const int DECELERATION_IN_STEPS_PER_SECOND = 500;

// crear objeto del motor paso a paso
ESP_FlexyStepper stepper;
int previousDirection = 1;
SemaphoreHandle_t emergencySemaphore;

// ISR del botón de emergencia: solo avisa a la tarea dedicada
void IRAM_ATTR emergencySwitchHandler()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(emergencySemaphore, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken)
  {
    portYIELD_FROM_ISR();
  }
}

// tarea de alta prioridad que realiza el paro de emergencia
void emergencyTask(void *p)
{
  for (;;)
  {
    if (xSemaphoreTake(emergencySemaphore, portMAX_DELAY) == pdTRUE)
    {
      stepper.emergencyStop();
    }
  }
}

void setup() 
{
  Serial.begin(115200);
  // configurar el pin del botón de emergencia como entrada con pull-up
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
  emergencySemaphore = xSemaphoreCreateBinary();
  xTaskCreate(emergencyTask, "emergencia", 2048, NULL, configMAX_PRIORITIES - 1, NULL);
  // adjuntar la interrupción al pin del botón
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_STOP_PIN), emergencySwitchHandler, RISING);
  // conectar y configurar el motor
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);

  // configurar velocidad y aceleración
  stepper.setSpeedInStepsPerSecond(SPEED_IN_STEPS_PER_SECOND);
  stepper.setAccelerationInStepsPerSecondPerSecond(ACCELERATION_IN_STEPS_PER_SECOND);
  stepper.setDecelerationInStepsPerSecondPerSecond(DECELERATION_IN_STEPS_PER_SECOND);
}

void loop() 
{
  if (stepper.getDirectionOfMotion() == 0)
  {
    delay(4000);
    previousDirection *= -1;
    stepper.setTargetPositionRelativeInSteps(DISTANCE_TO_TRAVEL_IN_STEPS * previousDirection);
  }
  // actualizar el motor de forma no bloqueante
  stepper.processMovement();
}
