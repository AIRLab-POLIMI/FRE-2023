// Use if you want to force the software SPI subsystem to be used for some reason (generally, you don't)
// #define FASTLED_FORCE_SOFTWARE_SPI
// Use if you want to force non-accelerated pin access (hint: you really don't, it breaks lots of things)
// #define FASTLED_FORCE_SOFTWARE_SPI
// #define FASTLED_FORCE_SOFTWARE_PINS
#include <FastLED.h>

///////////////////////////////////////////////////////////////////////////////////////////
//
// Move a white dot along the strip of leds.  This program simply shows how to configure the leds,
// and then how to turn a single pixel white and then off, moving down the line of pixels.
// 

// How many leds are in the strip?
#define NUM_LEDS 12

// For led chips like WS2812, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
// Clock pin only needed for SPI based chipsets when not using hardware SPI
#define DATA_PINR 3
#define DATA_PINC 5
#define DATA_PINL 10

// This is an array of leds.  One item for each led in your strip.
CRGB ledsR[NUM_LEDS];
CRGB ledsC[NUM_LEDS];
CRGB ledsL[NUM_LEDS];


// This function sets up the ledsand tells the controller about them
void setup() {
	// sanity check delay - allows reprogramming if accidently blowing power w/leds
   	delay(200);
    
    // FastLED.addLeds<WS2811, DATA_PINR, RGB>(ledsR, NUM_LEDS);
    // FastLED.addLeds<WS2811, DATA_PINC, RGB>(ledsC, NUM_LEDS);
    // FastLED.addLeds<WS2811, DATA_PINL, RGB>(ledsL, NUM_LEDS);

    FastLED.addLeds<WS2812B, DATA_PINR, GRB>(ledsR, NUM_LEDS);
    FastLED.addLeds<WS2812B, DATA_PINC, GRB>(ledsC, NUM_LEDS);
    FastLED.addLeds<WS2812B, DATA_PINL, GRB>(ledsL, NUM_LEDS);

    Serial.begin(9600);

}

// This function runs over and over, and is where you do the magic to light
// your leds.
void loop() {
   // Move a single white led 

   //int inByte = Serial.read();
   String teststr = Serial.readStringUntil('\n');
   teststr.trim();
   
   for(int i = 0; i < NUM_LEDS; i = i + 1) {
      // Turn our current led on to white, then show the leds
      if(teststr == "G/r"){
        ledsR[i] = CRGB::Green;
      }
      else if (teststr == "b/r"){
        ledsR[i] = CRGB::Black;
      }
      else if (teststr == "G/l"){
        ledsL[i] = CRGB::Green;
      }
      else if (teststr == "b/l"){
        ledsL[i] = CRGB::Black;
      }
      else{
        ledsR[i] = CRGB::Black;
        ledsL[i] = CRGB::Black;
        ledsC[i] = CRGB::Yellow;
      }
      
   }
    
      FastLED.show();


     
      //delay(100);

      // // Turn our current led back to black for the next loop around
      // for(int whiteLed = 0; whiteLed < NUM_LEDS; whiteLed = whiteLed + 1) {
      //     // Turn our current led on to white, then show the leds
      //     ledsR[whiteLed] = CRGB::Black;
      //     ledsC[whiteLed] = CRGB::Black;
      //     ledsL[whiteLed] = CRGB::Black;
      // }
      // FastLED.show();
      
      //delay(100);
}
