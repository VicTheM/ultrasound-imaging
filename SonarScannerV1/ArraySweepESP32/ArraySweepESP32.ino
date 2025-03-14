// Pin Definitions:
// The pins below are used for various connections, including ADC, GPIO, and JTAG.
// Pins: 4, 5, 12 (TDI), 13 (TCK), 14 (TMS), 15 (TDO), 16, 17,
//       18, 19, 21, 22, 23, 25, 26, 27, 0, 2, 32
// Pin 33 is used for enabling or standby mode.

// Set Arduino tasks to Core 0
#include <driver/adc.h>  // Include ADC driver

// Define constants
const long baud = 921600;  // Serial baud rate
const unsigned long cyclesPerSecond = 240000000; // ESP32 CPU cycles per second
const unsigned long cyclesPerMilliSecond = cyclesPerSecond / 1000;
const unsigned long cyclesPerMicroSecond = cyclesPerSecond / 1000000;
const unsigned long cyclesPerPhase = cyclesPerSecond / 40000; // 40kHz cycle frequency
const unsigned long cyclesPerPhaseHalf = cyclesPerPhase / 2;

// Define distance measurement parameters
const float meterPerSecond = 344.f;  // Speed of sound in air (m/s)
const float arrayWidthMm = 10 * 8;   // Width of the sensor array in mm
const float mmPerUs = meterPerSecond * 0.001f; // mm per microsecond
const long arrayWidthUs = long(arrayWidthMm / mmPerUs); 
const float startMm = 200;  // Initial distance in mm
const long startUs = long(startMm / mmPerUs); // Initial distance in microseconds

// Analog settings
const int analogBias = 0;

// GPIO pin configuration
const int enablePin = 33; // Enable pin
const int pinCount = 8;  // Number of active GPIO pins for the phased array
const int pulseLength = 20; // Pulse length in microseconds
const int pins[] = {4, 5, 12, 13, 14, 15, 16, 17}; // GPIO pin array

// Phase shift matrix for controlling waveforms
int phaseShifts[64][pinCount];

// Variables for signal processing
volatile int phaseShift = 0;
const int samplingRate = 20000; // Sampling rate for ADC
volatile int depth = 256;  // Depth of recorded samples
volatile int width = 64;   // Width of recorded samples
volatile int currentRec = 0; // Index for current recording buffer
volatile bool ready2send = false; // Flag indicating data readiness
int distance[2]; // Measured distances
int start[2];    // Start times
int recShift[2]; // Phase shifts for recorded data
short rec[2][64][256]; // Recorded waveform data

// Function to print a 4-bit hex value
void print4(unsigned long v)
{
	const char *hex = "0123456789abcdef";
	Serial.write(hex[v & 15]);
}

// Function to print an 8-bit hex value
void print8(unsigned long v)
{
	print4(v >> 4);
	print4(v);
}

// Function to print a 16-bit hex value
void print16(unsigned long v)
{
	print8(v >> 8);
	print8(v);
}

// Task for handling communication with the serial interface
void comTask(void *data)
{
	while(true)
	{
		// Wait until data is ready
		do { delay(1); } while(!ready2send);
		
		// Send recorded data over serial
		print16(width);
		print16(depth);
		print16(start[currentRec ^ 1]);
		print16(distance[currentRec ^ 1]);
		
		// Send phase-shifted signal data
		for(int j = 0; j < width; j++)
		{
			for(int i = 0; i < depth; i++)
			{
				print4(min((rec[currentRec ^ 1][j][i] * (i + 8)) / 64, 255) >> 4);
			}
		}
		Serial.println();

		// Process serial input commands
		static String s = "";
		while(Serial.available() > 0) 
		{
			char ch = Serial.read();
			if(ch == '\n')
			{
				sscanf(s.c_str(), "%d %d", &width, &depth);
				s = "";
				Serial.println("OK");
			}
			else
				s += ch;
		}
		ready2send = false;
	}
}

// Task for generating and capturing wave signals
void waveTask(void *param)
{
	long sum = 0;
	long avg = 2048; // Initial average ADC value

	while(true)
	{
		// Send pulse (400µs)
		digitalWrite(enablePin, 1);	// Enable output
		unsigned long t = 0;
		unsigned long ot = ESP.getCycleCount();

		while(true)
		{
			unsigned long ct = ESP.getCycleCount();
			unsigned long dt = ct - ot;
			ot = ct;
			t += dt;
			if(t >= 400 * cyclesPerMicroSecond) break;

			// Calculate phase shift and output signals
			unsigned long phase = t % cyclesPerPhase;
			unsigned long parallel1 = 0;
			unsigned long parallel0 = 0;

			for(int i = 0; i < pinCount; i++)
			{
				if((phaseShifts[phaseShift][i] + phase) % cyclesPerPhase > cyclesPerPhaseHalf)
					parallel1 |= 1 << pins[i];
				else
					parallel0 |= 1 << pins[i];
			}
			*(unsigned long*)GPIO_OUT_W1TS_REG = parallel1; // Set high
			*(unsigned long*)GPIO_OUT_W1TC_REG = parallel0; // Set low  
		}
		digitalWrite(enablePin, 0);	// Disable output
		
		// Wait for echo response (800µs delay)
		delayMicroseconds(startUs);
		adc1_config_width(ADC_WIDTH_BIT_12); // Set ADC resolution to 12-bit
		adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); // Set ADC attenuation
	  
		recShift[currentRec] = phaseShift;
		unsigned long t0 = ESP.getCycleCount();

		// Read ADC values and compute signal reflection
		for(int i = 0; i < depth; i++)
		{
			// int a = adc1_get_raw(ADC1_CHANNEL_6);
      // Mocked for now
      int a = 2048 + 1000 * sin(2 * PI * i / depth); // Simulated sine wave
			sum += a;
			rec[currentRec][phaseShift][i] = abs(a - avg);
		}

		// Calculate distance based on signal reflection time
		distance[currentRec] = (int)((ESP.getCycleCount() - t0) * mmPerUs / cyclesPerMicroSecond);
		start[currentRec] = startMm;

		// Update phase shift
		phaseShift = (phaseShift + 1) % width;

		// If all phase shifts are complete, switch recording buffer
		if(phaseShift == 0)
		{
			do { delay(0); } while(ready2send);
			currentRec ^= 1; // Switch recording buffer
			ready2send = true;
			while (ready2send) delay(1);
			avg = (sum / depth) / width;
			sum = 0;
		}
  	}
}

// Setup function
void setup() 
{
	Serial.begin(baud);
	Serial.println("Phased array setup!"); 

	// Set pin modes
	for(int i = 0; i < pinCount; i++)
		pinMode(pins[i], OUTPUT);
	pinMode(enablePin, OUTPUT);

	// Calculate phase shift values
	calculateWave();

	// Create tasks for communication and wave processing
	TaskHandle_t xHandle = NULL;
	xTaskCreatePinnedToCore(comTask, "Communication", 10000, 0, 2, &xHandle, 0);
	xTaskCreatePinnedToCore(waveTask, "Wave", 10000, 0, (2 | portPRIVILEGE_BIT), &xHandle, 1);
}

// Function to calculate phase shift values for phased array
void calculateWave()
{
	for(int i = 0; i < 64; i++)
	{
		float shift = (31 - i) / 64.0f;
		for(int j = 0; j < pinCount; j++)
		{
			float phase = j * shift;
			while(shift < 0) shift += 1;
			phaseShifts[i][j] = (int)(phase * cyclesPerPhase) % cyclesPerPhase;
		}
	}
}

// Loop function (unused, just a delay)
void loop() 
{
	delay(1000);
}
