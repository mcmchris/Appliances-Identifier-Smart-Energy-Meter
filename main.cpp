
/*
 * Project: Appliances Identifier Smart Energy Meter | Edge Impulse + Blues Wireless
 * Hardware: Nucleo-F767ZI + WiFi Notecard + Custom PCB
 * Author: Christopher Mendez Martinez
 * Code repository: https//
 * Tutorial guide: https//
 * Date: Nov 17th, 2022
 * Revision: 0.0.1
 */

// Libraries

#include "ei_run_classifier.h" // AI Model Library exported from Edge Impulse
#include "mbed.h"

#include "numpy.hpp"
#include <cstdint>
#include <string>

#define ADC_COUNTS (1 << 12) // ADC resolution

#define PRODUCT_UID "com.hotmail.mcmchris:mcmmeter" // Notehub device identifier
#define myProductID PRODUCT_UID

// Measuring task instance
Thread thread;

// Features timer instance
Timer t;

unsigned long previousMillis = 0;  // will store last time Notecard sent

// Shared resources semaphore
Semaphore nombre(1);

// Create a BufferedSerial object with a default baud rate.
static BufferedSerial serial_port(PF_7, PF_6); // On CN11 (TX, RX)

using namespace std::chrono; // allows to measure time in microseconds

float tiempo; // time keeping variable

static int64_t sampling_freq = 2523; // in Hz. Sensors sampling frequency (Same as Edge Impulse)
static int64_t time_between_samples_us = (1000000 / (sampling_freq - 1)); // Samples period

// Analog pins used of the Nucleo
AnalogIn ISignal(A1);  // CT sensor
AnalogIn VSignal(A0);  // PT sensor

static float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE]; // Sensor data array (1260)
                                                           // Data (252*5)

// Control variables
uint8_t seteo = 255, setted = 0, inicioRed = 0, lastseteo, prevseteo; // Digital potentiometer setup variables

unsigned int b = 100; // Auto range variable to control the digital potentiometer

int ready = 1;  // Control variable

// Code functions
void DigiPot(int x); // Adjust the digital potentiometer

void autoRango();    // Adjust current gain

void llenado(); // Fills the arrays with energy and raw datas to be processed and do inferences


void inferencia(); // Receive the data and outputs the appliance connected

float minReturn(
    float dataset[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE]); // returns minimun current value
                                                        
float maxReturn(
    float dataset[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE]); // returns maximun current value
                                                        
float map(float x, float in_min, float in_max, float out_min, float out_max); // Float numbers interpolation

void calcVI(unsigned int crossings, unsigned int timeout); // Calculate energy variables

// I2C peripheral instance
I2C i2c(PB_9, PB_8);


ei_impulse_result_t result = {0};

//--------------------------------------------------------------------------------------
// Energy measurement variables
//--------------------------------------------------------------------------------------

int SupplyVoltage = 3300; // power supply voltage

float realPower, apparentPower, powerFactor, Vrms, Irms, KWH; // Energy variables

// Calibration parameters
double VCAL = 165.892; // voltage
double ICAL = 0.096;   // current
double PHASECAL = 1.7; // phase

int sampleV; // Raw ADC voltage data
int sampleI; // Raw ADC current data

double lastFilteredV, filteredV, filteredI;  // ADC raw data minus 1.65v DC offset
double offsetV; // Voltage low pass filter output
double offsetI; // Voltage low pass filter output

double phaseShiftedV; // Voltage reference

double sqV, sumV, sqI, sumI, instP, sumP; // sq = square root, sum = Add , inst = instantaneous

int startV; // Starting voltage

int lastVCross, checkVCross; // zero corrsing count

// LED outputs
DigitalOut led3(LED3);
DigitalOut led2(LED2);

void UARTinit() {
  // We use the speed of 9600 because
  // the Notecard's RX/TX pins are always configured for that speed.
  // Set desired properties (9600-8-N-1).
  serial_port.set_baud(9600);
  serial_port.set_format(
      /* bits */ 8,
      /* parity */ BufferedSerial::None,
      /* stop bit */ 1);
}

// Measurement task (Stack size = 4096)
void medicion() {

  Timer integrator; // Integration timer init

  while (true) {

    nombre.acquire(); // Block the data to be used
    led2 = 1;

    integrator.start(); // Start counting

    calcVI(120, 2000); // Get energy variables

    led2 = 0;

    llenado(); //

    nombre.release(); // Free the data used

    integrator.stop();
    tiempo = duration_cast<microseconds>(integrator.elapsed_time()).count();
    integrator.reset();

    //printf("Duro = %f\n", tiempo);
    KWH += realPower / ((1000000.0 / tiempo) * 3600000.0); // Integrate power to extract energy

    if(Vrms < 10){
        printf("There's no energy!\r\n");
    }

  }
}

// Function to parse and send the Notecard data through serial
void NotecardSend(){

char message1[109];
      
      sprintf(message1,
              "{"
              "\"cmd\":\"note.add\""
              ","
              "\"sync\":true"
              ","
              "\"body\":{\"voltage\":%.2f,\"current\":%.2f,\"power\":%.2f,"
              "\"energy\":%.2f,\"pf\":%.2f}"
              "}\r\n",
              Vrms, Irms, realPower, KWH, powerFactor); 

      serial_port.write(message1, sizeof(message1));

      char message2[150];

      sprintf(message2,
              "{"
              "\"cmd\":\"note.add\""
              ","
              "\"sync\":true"
              ","
              "\"body\":{\"refri\":%.2f,\"fan\":%.2f,\"lightbulb\":%.2f,"
              "\"tv\":%.2f,\"ac\":%.2f,\"microwave\":%.2f,\"unknown\":%.2f,"
              "\"nothing\":%.2f}"
              "}\r\n",
              result.classification[4].value, result.classification[0].value,
              result.classification[2].value, result.classification[6].value,
              result.classification[5].value, result.classification[1].value,
              result.classification[3].value, result.anomaly); // 22

      serial_port.write(message2, sizeof(message2));
}


int main() {

  UARTinit();

  char msg1[] = "{\"cmd\":\"hub.set\",\"product\":\"" myProductID "\"}\r\n";
  serial_port.write(msg1, sizeof(msg1));
  char msg2[] = "{\"cmd\":\"hub.set\",\"mode\":\"continuous\"}\r\n";
  serial_port.write(msg2, sizeof(msg2));

  // Measurement task declaration
  thread.start(medicion);
  thread.set_priority(osPriorityNormal);

  i2c.frequency(400000); // I2C frequency

  // Start counting
  t.start();

  printf("Edge Impulse standalone inferencing (Mbed)\n");

  DigiPot(seteo); // INAMP setted to max gain

  while (1) {

    // Wait for the data to be stored to start the inference
    if (nombre.try_acquire() == false && Vrms > 10) {  // not happening if there's no energy

      
      inferencia();  // start the inference

      printf("Vrms = %f V; Irms = %f I; P = %f W; S = %f VA; FP = %f; KWH = %f\n",
          Vrms, Irms, realPower, apparentPower, powerFactor, KWH);

      nombre.release();
    }

    unsigned long currentMillis = HAL_GetTick(); // Start counting time to send every 10 seconds

    if ((currentMillis - previousMillis) >= 10000 && Vrms > 10) {
        previousMillis = currentMillis;

        printf("Notecard sending\r\n");
        NotecardSend();  // WiFi send to the cloud
    }


    ThisThread::sleep_for(1s);
  }
}

void inferencia() {
  signal_t features_signal;

  numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE,
                            &features_signal);

  // impulse call
  EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);
  printf("run_classifier returned: %d\n", res);

  // print inference results
  printf("[");
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    printf("%.5f", result.classification[ix].value);
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    printf(", ");
#else
    if (ix != EI_CLASSIFIER_LABEL_COUNT - 1) {
      printf(", ");
    }
#endif
  }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
  printf("%.3f", result.anomaly);
#endif
  printf("]\n");

  if (result.anomaly > 0.59) {
    result.classification[0].value = 0;
    result.classification[1].value = 0;
    result.classification[2].value = 0;
    result.classification[3].value = 0;
    result.classification[4].value = 0;
    result.classification[5].value = 0;
    result.classification[6].value = 0;
  }
}

// capture data and auto calibrate
void llenado() {
  while (1) {
    lastseteo = seteo;
    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
         ix += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME) {
      int64_t next_tick = t.elapsed_time().count() + time_between_samples_us;

      // copy meter data into the features array
      features[ix + 0] = VSignal.read() * 4095;
      features[ix + 1] = ISignal.read() * 4095;
      features[ix + 2] = Irms;
      features[ix + 3] = realPower;
      features[ix + 4] = powerFactor;

      while (t.elapsed_time().count() < next_tick) {
        // 400us delay to achieve Edge Impulse frequency 2523 hz
      }
    }
    autoRango();
    if (seteo == lastseteo) {
      break;
    }
  }
}

void calcVI(unsigned int crossings, unsigned int timeout) {

  unsigned int crossCount = 0; // zero crossings count
  unsigned int numberOfSamples = 0; // samples count
  //-------------------------------------------------------------------------------------------------------------------------
  // 1) Waits for the waveform to be close to 'zero' (mid-scale adc) part in sin curve.
  //-------------------------------------------------------------------------------------------------------------------------
  unsigned long start = HAL_GetTick(); // makes sure it doesnt get stuck in the loop if there is an error.

  while (1) // wait for the voltage to be in range
  {
    startV = VSignal.read() * 4095; // using the voltage waveform
    if ((startV < (ADC_COUNTS * 0.51)) && (startV > (ADC_COUNTS * 0.49)))
      break; // start from cero
    if ((HAL_GetTick() - start) > timeout) {
      break;
    }
  }

  //-------------------------------------------------------------------------------------------------------------------------
  // 2) Main measurement loop
  //-------------------------------------------------------------------------------------------------------------------------
  start = HAL_GetTick();

  while ((crossCount < crossings) && ((HAL_GetTick() - start) < timeout)) {
    numberOfSamples++;         // Count number of times looped.
    lastFilteredV = filteredV; // Used for delay/phase compensation

    //-----------------------------------------------------------------------------
    // A) Reading raw voltage and current takes
    //-----------------------------------------------------------------------------
    sampleV = VSignal.read() * 4095; // Read in raw voltage signal
    sampleI = ISignal.read() * 4095; // Read in raw current signal

    //-----------------------------------------------------------------------------
    // B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset,
	//     then subtract this - signal is now centred on 0 counts.
    //-----------------------------------------------------------------------------
    offsetV = offsetV + ((sampleV - offsetV) / ADC_COUNTS);
    filteredV = sampleV - offsetV;
    offsetI = offsetI + ((sampleI - offsetI) / ADC_COUNTS);
    filteredI = sampleI - offsetI;

    //-----------------------------------------------------------------------------
    // C) Voltage RMS method
    //-----------------------------------------------------------------------------
    sqV = filteredV * filteredV; // 1) squaring voltage
    sumV += sqV;                 // 2) add

    //-----------------------------------------------------------------------------
    // D) Current RMS method
    //-----------------------------------------------------------------------------
    sqI = filteredI * filteredI; // 1) squaring current
    sumI += sqI;                 // 2) add

    //-----------------------------------------------------------------------------
    // E) Phase calibration
    //-----------------------------------------------------------------------------
    phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV);

    //-----------------------------------------------------------------------------
    // F) Instantaneous power calc
    //-----------------------------------------------------------------------------
    instP = phaseShiftedV * filteredI; // P = V*I
    sumP += instP;                     // add

    //-----------------------------------------------------------------------------
    // G) Find the number of times the voltage has crossed the initial voltage
	//    - every 2 crosses we will have sampled 1 wavelength
	//    - so this method allows us to sample an integer number of half wavelengths which increases accuracy
    //-----------------------------------------------------------------------------
    lastVCross = checkVCross;
    if (sampleV > startV) {
      checkVCross = 1;
    } else {
      checkVCross = 0;
    }
    if (numberOfSamples == 1) {
      lastVCross = checkVCross;
    }

    if (lastVCross != checkVCross) {
      crossCount++;
    }
  }

  //-------------------------------------------------------------------------------------------------------------------------
  // 3) Post loop calculations
  //-------------------------------------------------------------------------------------------------------------------------
  //Calculation of the root of the mean of the voltage and current squared (rms)
  //Calibration coefficients applied.

  double V_RATIO = VCAL * ((SupplyVoltage / 1000.0) / (ADC_COUNTS));
  Vrms = V_RATIO * sqrt(sumV / numberOfSamples);

  double I_RATIO = ICAL * ((SupplyVoltage / 1000.0) / (ADC_COUNTS));
  Irms = I_RATIO * sqrt(sumI / numberOfSamples);

  //Calculation power values
  realPower = V_RATIO * I_RATIO * sumP / numberOfSamples;
  apparentPower = Vrms * Irms;
  powerFactor = realPower / apparentPower;

  //Reset accumulators
  sumV = 0;
  sumI = 0;
  sumP = 0;

  //--------------------------------------------------------------------------------------
}

//--------------------------------------------------------------------------------------
// Digital potentiometer function
//--------------------------------------------------------------------------------------
void DigiPot(int x) {
  const int addr8bit = 0x2f << 1; // 8bit I2C address
  char info[2];
  info[0] = 0x00;
  info[1] = x;
  i2c.write(addr8bit, info, 2, false);
}

//--------------------------------------------------------------------------------------
// INAMP current autorange function
//--------------------------------------------------------------------------------------
void autoRango() {
  float minval = minReturn(features);
  float maxval = maxReturn(features);

  if (minval <= 0 || maxval > 3000) {
    setted = 0;
    seteo--;
    if (seteo <= 0) {
      seteo = 0;
    }
    DigiPot(seteo);
  } else {
    setted = 1;
  }
  b = maxval - minval;
  if (setted == 1 && b < 1300) {
    minval = minReturn(features);
    seteo = 255;
    DigiPot(seteo);
  }
  //printf("ICAL = %f, SETEO = %d\n", ICAL, seteo);

  // Manual calibration of current on each gain step (This code block worth millions)
  switch (seteo) {
  case 255:
    ICAL = 0.096;
    break;
  case 254:
    ICAL = 0.5151;
    break;
  case 253:
    ICAL = 0.915;
    break;
  case 252:
    ICAL = 1.32;
    break;
  case 251:
    ICAL = 1.72;
    break;
  case 250:
    ICAL = 2.115;
    break;
  case 249:
    ICAL = 2.51;
    break;
  case 248:
    ICAL = 2.91;
    break;
  case 247:
    ICAL = 3.3;
    break;
  case 246:
    ICAL = 3.665;
    break;
  case 245:
    ICAL = 4.05;
    break;
  case 244:
    ICAL = 4.43;
    break;
  case 243:
    ICAL = 4.815;
    break;
  case 242:
    ICAL = 5.195;
    break;
  case 241:
    ICAL = 5.565;
    break;
  case 240:
    ICAL = 5.89;
    break;
  case 239:
    ICAL = 6.25;
    break;
  case 238:
    ICAL = 6.635;
    break;
  case 237:
    ICAL = 7.01;
    break;
  case 236:
    ICAL = 7.35;
    break;
  case 235:
    ICAL = 7.73;
    break;
  case 234:
    ICAL = 8.085;
    break;
  case 233:
    ICAL = 8.4;
    break;
  case 232:
    ICAL = 8.805;
    break;
  case 231:
    ICAL = 9.12;
    break;
  case 230:
    ICAL = 9.435;
    break;
  case 229:
    ICAL = 9.795;
    break;
  case 228:
    ICAL = 10.135;
    break;
  case 227:
    ICAL = 10.37;
    break;
  case 226:
    ICAL = 10.72;
    break;
  case 225:
    ICAL = 11.03;
    break;
  case 224:
    ICAL = 11.375;
    break;
  case 223:
    ICAL = 11.72;
    break;
  case 222:
    ICAL = 12.025;
    break;
  case 221:
    ICAL = 12.405;
    break;
  case 220:
    ICAL = 12.72;
    break;
  case 219:
    ICAL = 13.065;
    break;
  case 218:
    ICAL = 13.37;
    break;
  case 217:
    ICAL = 13.67;
    break;
  case 216:
    ICAL = 13.98;
    break;
  case 215:
    ICAL = 14.37;
    break;
  case 214:
    ICAL = 14.595;
    break;
  case 213:
    ICAL = 14.915;
    break;
  case 212:
    ICAL = 14.96;
    break;
  case 211:
    ICAL = 15.28;
    break;
  case 210:
    ICAL = 15.56;
    break;
  case 209:
    ICAL = 15.85;
    break;
  case 208:
    ICAL = 16.145;
    break;
  case 207:
    ICAL = 16.435;
    break;
  case 206:
    ICAL = 16.73;
    break;
  case 205:
    ICAL = 17.02;
    break;

  default:
    ICAL = map(seteo, 213, 205, 14.915, 17.02); // Interpolate unknown linear
  }
}


//--------------------------------------------------------------------------------------
// Find minimum and maximum current for autorange function
//--------------------------------------------------------------------------------------
float minReturn(float dataset[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE]) {
  int mini = 2030;
  for (int z = 1; z < 250; z += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME) {
    if (dataset[z] < mini) {
      mini = dataset[z];
    }
  }
  return mini;
}
float maxReturn(float dataset[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE]) {
  int maxi = 2030;
  for (int z = 1; z < 250; z += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME) {
    if (dataset[z] > maxi) {
      maxi = dataset[z];
    }
  }
  return maxi;
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}