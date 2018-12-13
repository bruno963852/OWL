/*!
 * @file main.cpp
 *
 * @mainpage OWL Ultrassonic car detector
 *
 * @section intro_sec Introduction
 *
 * This is the code for a sensor board using a ultrassonic sensor to detect
 * the presence of objects under a parking lot gate, and should be user to
 * prevent the gat for going down when such objects are detected
 *
 * @section dependencies Dependencies
 *
 * This code depends on <a href="https://github.com/ErickSimoes/Ultrasonic</a> 
 * installed in your system I recommend installing it via platformio
 *
 * @section author Author
 *
 * Written by Bruno Martins and Rebeca Matos
 *
 * @section license License
 *
 * BSD license, all text here must be included in any istribution.
 *
 */

#include <Arduino.h>
#include <assert.h>
#include <Ultrasonic.h>

#define TRIGGER_PIN 2 ///< The pin connected to the ultrassonic sensor trigger
#define ECHO_PIN 3    ///< The pin connected to the ultrassonic sensor echo
#define ULTRASONIC_TIMEOUT 20000UL ///< The timeout for the ultrasonic module
#define DISTANCE_SAMPLES 2 ///< The ammount of sampes to collect to calculate the distance

#define DISTANCE_TRIMPOT_INPUT_PIN A1  ///< The analog pin connected to the trimpot center
#define DISTANCE_TRIMPOT_MAX_VALUE 300 ///< The distance in cm the represents the pot in max state
#define DISTANCE_TRIMPOT_MIN_VALUE 0   ///< The distance in cm the represents the pot in min state

#define DELAY_TRIMPOT_INPUT_PIN A0  ///< The analog pin connected to the trimpot center
#define DELAY_TRIMPOT_MAX_VALUE 5000 ///< The distance in cm the represents the pot in max state
#define DELAY_TRIMPOT_MIN_VALUE 0   ///< The distance in cm the represents the pot in min state

//#define RELEASE_DELAY 3000 ///< Delay for releasing the detection
#define DETECTION_DELAY 200 ///< Dalay for afirming a detection

#define BLUE_LED_PIN 10  ///< The pin connected to the blue leg on the RGB led (common anode)
#define GREEN_LED_PIN 9 ///< The pin connected to the green leg on the RGB led (common anode)
#define RED_LED_PIN 11    ///< The pin connected to the  leg on the RGB led (common anode)

#define OUTPUT_SIGNAL_PIN 12 ///< The pin connected to the autoput signal of the board (1 for object detected, 0 for no object)

#define SERIAL_BAUD_RATE 115200 ///< The baudate of the serial port

#define LOG_EVENTS 
#define LOG_MEASUREMENTS

Ultrasonic g_ultrasonic(TRIGGER_PIN, ECHO_PIN, ULTRASONIC_TIMEOUT); ///< The ultrassonic sensor object (global)

unsigned long g_detection_time = 0; ///< Stores the moment of the detection

/**
 * @brief Enum for controlling the state of the device
 * 
 */
enum DetectionState {
  Released,
  Detecting,
  Detected,
  Releasing
};

DetectionState g_detection_state = Detected; ///< The detection state of the sensor (global)

/**
 * @brief Set the rgb LED state
 * 
 * @param red_value the value of the red color (0 <= value < 256)
 * @param green_value the value of the green color (0 <= value < 256)
 * @param blue_value the value of the blue color (0 <= value < 256)
 */
void setLed(int red_value, int green_value, int blue_value)
{
  assert(red_value >= 0 && green_value >= 0 && blue_value >= 0 &&
         red_value < 256 && green_value < 256 && blue_value < 256);

  analogWrite(RED_LED_PIN, 255 - red_value);
  analogWrite(GREEN_LED_PIN, 255 - green_value);
  analogWrite(BLUE_LED_PIN, 255 - blue_value);
}

/**
 * @brief Set the rgb LED state
 * 
 * @param rgb_values vector with the 3 values for the led colors [r, g, b] (0 < value < 255)
 */
void setLed(int *rgb_values)
{
  assert(rgb_values[0] >= 0 && rgb_values[1] >= 0 && rgb_values[2] >= 0 &&
         rgb_values[0] < 256 && rgb_values[1] < 256 && rgb_values[2] < 256);

  analogWrite(RED_LED_PIN, 255 - rgb_values[0]);
  analogWrite(GREEN_LED_PIN, 255 - rgb_values[1]);
  analogWrite(BLUE_LED_PIN, 255 - rgb_values[2]);
}

/**
 * @brief Get the Trimpot Distance Value
 * 
 * @return int the distance in cm represented by the trimpot value
 */
int getTrimpotDistanceValue()
{
  return (int)map(analogRead(DISTANCE_TRIMPOT_INPUT_PIN), 1023, 0, 
                  DISTANCE_TRIMPOT_MAX_VALUE, DISTANCE_TRIMPOT_MIN_VALUE);
}

/**
 * @brief Get the Trimpot Delay Value
 * 
 * @return int the delay in miliseconds represented by the trimpot value
 */
int getTrimpotDelayValue()
{
  return (int)map(analogRead(DELAY_TRIMPOT_INPUT_PIN), 1023, 0, 
                  DELAY_TRIMPOT_MAX_VALUE, DELAY_TRIMPOT_MIN_VALUE);
}

/**
 * @brief Takes various samples of distance and return the medium
 * 
 * @return int the distance measured
 */
int measureDistance() {
  long distance_total = 0;
  int distance_quantities = 0;
  for (int i = 0; i < DISTANCE_SAMPLES; i++) {
    int dist = g_ultrasonic.read();
    if (dist <= DISTANCE_TRIMPOT_MAX_VALUE) {
      distance_total += dist;
      distance_quantities++;
    } else {
      distance_total += 300;
      distance_quantities++;
    }
    delay(10);
  }
  return (int) distance_total / distance_quantities;
}

/**
 * @brief Process the released state of the device
 * 
 * @param has_obstacle if has an obstacle
 */
void processReleasedState(bool has_obstacle) {
  if (has_obstacle) {
    g_detection_time = millis();
    g_detection_state = Detecting;
    setLed(0, 255, 0); //GREEN ONLY
    #ifdef LOG_EVENTS
    Serial.println("Detecting");
    #endif
  } else {
    setLed(0, 255, 0); //GREEN ONLY
  }
}

/**
 * @brief Process the detecting state of the device
 * 
 * @param has_obstacle if has an obstacle
 */
void processDetectingState(bool has_obstacle)
{
  if (has_obstacle) {
    if (g_detection_time + DETECTION_DELAY <= millis()) {
      #ifdef LOG_EVENTS
      Serial.println("Still Detecting");
      #endif
      setLed(0, 255, 0); //GREEN ONLY
    } else {
      g_detection_state = Detected;
      digitalWrite(OUTPUT_SIGNAL_PIN, HIGH);
      #ifdef LOG_EVENTS
      Serial.println("Detected!");
      #endif
      setLed(255, 0, 0); //RED ONLY
    }
  } else {
    setLed(0, 255, 0); //GREEN ONLY
    g_detection_state = Released;
    #ifdef LOG_EVENTS
    Serial.println("False Detection, Released!");
    #endif
  }
}

/**
 * @brief Process the detected state of the device
 * 
 * @param has_obstacle if has an obstacle
 */
void processDetectedState(bool has_obstacle) {
  if (has_obstacle) {
    setLed(255, 0, 0); //RED_ONLY
  } else {
    g_detection_time = millis();
    g_detection_state = Releasing;
    #ifdef LOG_EVENTS
    Serial.println("Releasing...");
    #endif
  }
}

/**
 * @brief Process the releasing state of the device
 * 
 * @param has_obstacle if has an obstacle
 */
void processReleasingState(bool has_obstacle, int release_delay) {
  if (has_obstacle) {
    setLed(255, 0, 0); //RED ONLY
    g_detection_state = Detected;
    #ifdef LOG_EVENTS
    Serial.println("Another Detection, Releasing Canceled!");
    #endif
  } else {
    if (g_detection_time + release_delay >= millis()) {
      #ifdef LOG_EVENTS
      Serial.println("Still Releasing");
      #endif
      setLed(0, 0, 255); //Blue ONLY
    } else {
      g_detection_state = Released;
      digitalWrite(OUTPUT_SIGNAL_PIN, LOW);
      #ifdef LOG_EVENTS
      Serial.println("Released!");
      #endif
      setLed(0, 255, 0); //GREEN ONLY
    }
  }
}

/**
 * @brief called by arduino before runing the loop
 * 
 */
void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(OUTPUT_SIGNAL_PIN, OUTPUT);
  setLed(0,0,0);
  g_detection_time = millis();
}

/**
 * @brief called by arduino inside a infinite loop
 * 
 */
void loop()
{
  int measured_distance = measureDistance();
  int trimpot_distance_value = getTrimpotDistanceValue();
  int trimpot_delay_value = getTrimpotDelayValue();

  #ifdef LOG_MEASUREMENTS
  Serial.print("Distance:\t");
  Serial.print(measured_distance);
  Serial.print("\t\t");
  Serial.print("Pot Distance:\t");
  Serial.print(trimpot_distance_value);
  Serial.print("\t");
  Serial.print("Pot Delay:\t");
  Serial.print(trimpot_delay_value);
  Serial.print("\t");
  Serial.print("Detection:\t");
  Serial.print(g_detection_state);
  Serial.println();
  #endif

  bool has_obstacle = measured_distance <= trimpot_distance_value;

  switch (g_detection_state) {
    case Released:
      processReleasedState(has_obstacle);
      break;
    case Detecting:
      processDetectingState(has_obstacle);
      break;
    case Detected:
    processDetectedState(has_obstacle);
      break;
    case Releasing:
    processReleasingState(has_obstacle, trimpot_delay_value);
      break;
  }
}

/**
 * @brief handle diagnostic informations given by assertion and abort program execution:
 * 
 * @param __func current fucntion
 * @param __file current file
 * @param __lineno current line
 * @param __sexp ???
 */
void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp)
{
  // transmit diagnostic informations through serial link.
  Serial.println(__func);
  Serial.println(__file);
  Serial.println(__lineno, DEC);
  Serial.println(__sexp);
  Serial.flush();
  // abort program execution.
  abort();
}