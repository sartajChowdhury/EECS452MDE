//ECG_Final.ino: This is an Arduino sketch and it is responsible for sampling the amplified ECG signal from the analog circuit at 300 Hz. 
//This sketch was flashed onto an ESP-32-WROVER-E for this project.
//To see more about the operation of this file view OVERVIEW.txt

#include <Filters.h>
#include <Filters/IIRFilter.hpp>

#define NUM_SAMPLES 900      // Total number of samples to acquire
#define SAMPLING_RATE 300     // Desired sampling rate in Hz
#define WINDOW_SIZE 5         // Size of the moving window for integration

//Number of fractional bits
#define FRACTIONAL_BITS 16
typedef int32_t fixed_point;
// Define a constant to represent the fractional multiplier
#define FRACTIONAL_MULTIPLIER (1 << FRACTIONAL_BITS)

//Macros for converting between fixed-point and floating-point
#define FLOAT_TO_FIXED(x) ((fixed_point)((x) * FRACTIONAL_MULTIPLIER))
#define FIXED_TO_FLOAT(x) ((float)(x) / FRACTIONAL_MULTIPLIER)

const fixed_point sampFreq = 300;
const float PERIOD = (1.0 / sampFreq);
int analogPin = 32;
const int sampleIntervalMs = 1000 / 300; // Sampling interval in milliseconds for 300 Hz

int sampleIndex = 0;  // Current sample index
int totalSamples = 0;

//Arrays for storing panTompkins calculated values
fixed_point panTompkins[NUM_SAMPLES];
fixed_point Squared[NUM_SAMPLES];

//Peak Estimation Constants
fixed_point highThreshold = 85; // Define based on signal characteristics
fixed_point lowThreshold = 42; // Define based on signal characteristics
int overallPeak = 155; // This could be the maximum value seen during a calibration phase or set to a fixed value initially
fixed_point signalPeakEstimate = 155;
fixed_point noisePeakEstimate = 42;
int peakCount = 0;
int lastPeak = 0;

int debugHeartRate = 0; //set to 0 to turn off prints for heart rate


//FIR Filter Coefficients and Delay Line
#define NUM_TAPS 201 // Number of filter taps (length of the filter)
float b[NUM_TAPS] = { -0.000299, -0.000229, -0.000021, 0.000222, 0.000367, 0.000318, 0.000079, -0.000241, -0.000467, -0.000454, -0.000173, 0.000257, 0.000606, 0.000655, 0.000324, -0.000254, -0.000780, -0.000932, -0.000557, 0.000210, 0.000978, 0.001292, 0.000893, -0.000095, -0.001179, -0.001734, -0.001354, -0.000125, 0.001352, 0.002244, 0.001949, 0.000480, -0.001459, -0.002797, -0.002682, -0.001001, 0.001457, 0.003358, 0.003539, 0.001706, -0.001302, -0.003878, -0.004497, -0.002607, 0.000954, 
0.004306, 0.005515, 0.003696, -0.000379, -0.004585, -0.006541, -0.004953, -0.000443, 0.004660, 0.007512, 0.006339, 0.001518, -0.004483, -0.008362, -0.007800, -0.002833, 0.004018, 0.009022, 0.009270, 0.004358, -0.003244, -0.009428, -0.010671, -0.006040, 0.002160, 0.009526, 0.011925, 0.007816, -0.000785, -0.009279, -0.012954, -0.009604, -0.000841, 0.008667, 0.013688, 0.011319, 0.002657, -0.007693, -0.014071, -0.012873, -0.004587, 0.006382, 0.014064, 0.014182, 0.006545, -0.004783, -0.013651, 
-0.015174, -0.008437, 0.002963, 0.012840, 0.015794, 0.010171, -0.001004, -0.011663, 0.984248, -0.011663, -0.001004, 0.010171, 0.015794, 0.012840, 0.002963, -0.008437, -0.015174, -0.013651, -0.004783, 0.006545, 0.014182, 0.014064, 0.006382, -0.004587, -0.012873, -0.014071, -0.007693, 0.002657, 0.011319, 0.013688, 0.008667, -0.000841, -0.009604, -0.012954, -0.009279, -0.000785, 0.007816, 0.011925, 0.009526, 0.002160, -0.006040, -0.010671, -0.009428, -0.003244, 0.004358, 0.009270, 0.009022, 
0.004018, -0.002833, -0.007800, -0.008362, -0.004483, 0.001518, 0.006339, 0.007512, 0.004660, -0.000443, -0.004953, -0.006541, -0.004585, -0.000379, 0.003696, 0.005515, 0.004306, 0.000954, -0.002607, -0.004497, -0.003878, -0.001302, 0.001706, 0.003539, 0.003358, 0.001457, -0.001001, -0.002682, -0.002797, -0.001459, 0.000480, 0.001949, 0.002244, 0.001352, -0.000125, -0.001354, -0.001734, -0.001179, -0.000095, 0.000893, 0.001292, 0.000978, 0.000210, -0.000557, -0.000932, -0.000780, -0.000254, 
0.000324, 0.000655, 0.000606, 0.000257, -0.000173, -0.000454, -0.000467, -0.000241, 0.000079, 0.000318, 0.000367, 0.000222, -0.000021, -0.000229, -0.000299};

fixed_point b_fixed[NUM_TAPS] = {0};
fixed_point x_fixed[NUM_TAPS] = {0}; //Input Delay Line

fixed_point filteredData[NUM_SAMPLES] = {0}; //Array for storing filtered ECG data

//Bandpass Filter Coefficients
IIRFilter<6,6,float> lowFilt = {{0.459365131515558,1.10367877429011,1.78651506551963,1.78651506551963,1.10367877429011,0.459365131515558},{1,1.23625833162127,2.26245257519559,1.22247053019850,0.918249898937826,0.0596866066974345}};
IIRFilter<4,4,float> highFilt = {{0.993155250710026,-2.97938142078140,2.97938142078140,-0.993155250710026},{1,-2.98620531684655,2.97253730286465,-0.986330723271646}};

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(5000); //Delay at begin to avoid transients at startup
  filter_float_to_fixed(b); //Convert Notch Filter coefficients to fixed point
  Serial.println("START");
}

void loop() {
  unsigned long startTime = millis(); // Record the start time of each loop iteration

  //Read analog ECG Value
  float sensorValue = analogRead(analogPin) / 4095.0f * 3300.0f;
  //Apply Notch Filter
  filteredData[sampleIndex] = FLOAT_TO_FIXED(apply_bandpass_filter(sensorValue));
  filteredData[sampleIndex] = apply_notch_filter(filteredData[sampleIndex]);
  Serial.println(filteredData[sampleIndex]);

  //Handle edge case for sampleIndex == 1
  if (sampleIndex == 1) {
    panTompkins[0] = 0;
    panTompkins[1] = FLOAT_TO_FIXED((filteredData[sampleIndex] - filteredData[sampleIndex - 1]) / 5.0f);
    
    //Compute Square
    Squared[sampleIndex] = computeSquare(panTompkins[sampleIndex]);
    
    //Compute Integral
    computeIntegration(Squared, panTompkins, 0);
  }

  //Regular Case
  //Apply delay of 2 to each sample because derivative needs to look two values into the future
  if (sampleIndex >= 4) {
    if (sampleIndex < NUM_SAMPLES) {
      //Compute Derivative
      panTompkins[sampleIndex - 2] = computeDerivative(filteredData[sampleIndex - 1], filteredData[sampleIndex], filteredData[sampleIndex - 3], filteredData[sampleIndex - 4] );
    }

    //Compute Square
    Squared[sampleIndex - 2] = computeSquare(panTompkins[sampleIndex - 2]);

    //Compute Integral
    computeIntegration(Squared, panTompkins, 1);

    //Edge cases at end of NUM_SAMPLES
    if (sampleIndex == NUM_SAMPLES - 2) {
      panTompkins[NUM_SAMPLES - 2] = FLOAT_TO_FIXED((filteredData[NUM_SAMPLES - 2] - filteredData[NUM_SAMPLES - 3]) / 10.0f);
      Squared[NUM_SAMPLES - 2] = computeSquare(panTompkins[NUM_SAMPLES - 2]);
      computeIntegration(Squared, panTompkins, 0);
    }

    if (sampleIndex == NUM_SAMPLES - 1) {
      panTompkins[NUM_SAMPLES - 1] = FLOAT_TO_FIXED((filteredData[NUM_SAMPLES - 1] - filteredData[NUM_SAMPLES - 2]) / 10.0f);
      Squared[NUM_SAMPLES - 1] = computeSquare(panTompkins[NUM_SAMPLES - 1]);
      computeIntegration(Squared, panTompkins, 0);
    }
    
    //Detect Peaks
    detectPeaks(panTompkins);
  }
  //only calculate heart rate if a new peak has been detected
  if (lastPeak != peakCount) {
    if (debugHeartRate) {
      Serial.print("Sample Index: ");
      Serial.print(sampleIndex - 2);
      Serial.print("  Peak Count: ");
      Serial.println(peakCount);
    }
    calculateHeartRate();
  }
  lastPeak = peakCount; //update most recent peak count

  //Delay to maintain desired sampling frequency
  unsigned long elapsedTime = millis() - startTime;
  if (elapsedTime < sampleIntervalMs) {
    delay(sampleIntervalMs - elapsedTime);
  }
  else {
    Serial.println("TIMING VIOLATED");
  }
  sampleIndex++;
  totalSamples++;

  //uncomment for live testing
  if (sampleIndex >= NUM_SAMPLES) {
    sampleIndex = 0;
  }
  

}

//Compute derivative
fixed_point computeDerivative(fixed_point future, fixed_point doubleFuture, fixed_point previous, fixed_point doublePrevious) {
  float T = 1.0f;
  fixed_point coefficient = FLOAT_TO_FIXED(1.0 / (8.0 * T));
  //float T = PERIOD;
  return coefficient * (-doublePrevious - 2 * previous + 2 * future + doubleFuture);
}

//Compute Integral
//If delayed is TRUE then delay sampleIndex by 2, else no delay
//In this case, the sampleIndex represents the last value in the integration window
void computeIntegration(fixed_point input[], fixed_point output[], bool delayed) {
  float sum = 0;
  float Window = 0;
  int index = 0;

  if (delayed) {
    index = sampleIndex - 2;
  }
  else {
    index = sampleIndex;
  }

  if (index == 0) {
    Window = 1;
  }
  else if (index < WINDOW_SIZE) {
    Window = index;
  }
  else {
    Window = WINDOW_SIZE;
  }
  for (int i = index - Window + 1; i <= index; i++) {
    sum = sum + input[i];
  }
  output[index] = FLOAT_TO_FIXED(sum / Window);
}

//Compute square
fixed_point computeSquare(fixed_point current) {
  // Compute the squared values of the signal
  return (current * current);
}

//Detect Peaks Function
void detectPeaks(volatile fixed_point signal[]) {
  float fractionSignal = 0.125; // Example fraction for signal peak estimate update
  float fractionNoise = 0.125; // Example fraction for noise peak estimate update
  int index = sampleIndex - 2; //the purpose of this is so we can use "future" values for our calculations

  //first NUM_TAPS samples will not be valid because filter will be getting set
  //uncomment for live testing
  if (index < NUM_TAPS) {
    return;
  }

  if (signal[index] > overallPeak) {
    overallPeak = signal[index];
  }
  if (signal[index - 1] > highThreshold) {
    //count peaks
    if (index > 2  && index < NUM_SAMPLES) {
      if ((signal[index - 1] < signal[index]) && (signal[index + 1] < signal[index]) ) {
        peakCount++;
      }
    }
    // Update signal peak estimate
    signalPeakEstimate = FLOAT_TO_FIXED(fractionSignal * overallPeak + (1 - fractionSignal) * signalPeakEstimate);
  }
  else if (signal[index] > lowThreshold) {
    // Update noise peak estimate
    noisePeakEstimate = FLOAT_TO_FIXED(fractionNoise * overallPeak + (1 - fractionNoise) * noisePeakEstimate);
  }

  // Update threshold values
  highThreshold = FLOAT_TO_FIXED(noisePeakEstimate + fractionSignal * (signalPeakEstimate - noisePeakEstimate));
  lowThreshold = FLOAT_TO_FIXED(0.5 * highThreshold);
}

//Calculate heart rate
void calculateHeartRate() {
  float currentTime = (totalSamples * PERIOD) / 60.0f; //calculates minutes passed
  float BPM = peakCount / currentTime;
  if (debugHeartRate) {
    Serial.print("Beats Per Minute: ");
    Serial.println(BPM);
  }
}

//Convert filter coefficients from float to fixed point values
void filter_float_to_fixed(float filter_array[]){
  for(int i = 0; i < NUM_TAPS; i++){
    b_fixed[i] = FLOAT_TO_FIXED(filter_array[i]);
  }
}

//Apply notch filter
fixed_point apply_notch_filter(fixed_point input) {
  fixed_point output = 0;

  //Shift input values in the delay line
  for (int i = NUM_TAPS - 1; i > 0; i--) {
    x_fixed[i] = x_fixed[i - 1];
  }

  //Update current input value
  x_fixed[0] = input;

  //Apply the FIR filter
  for (int i = 0; i < NUM_TAPS; i++) {
    output += b_fixed[i] * x_fixed[i];
  }

  return output;
}

//Apply bandpass filter
float apply_bandpass_filter(float input) {
  return lowFilt(highFilt(input));
}
