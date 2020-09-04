/*  PPM (pulse position modulation) sampling done in hardware via timer1.
    Sampling is done via digital pin 8.
    
    Code below supports frame buffering, which is used to protect the controller from
    errors introduced (from simple noise to complete RX failure), if any of the channels in a single
    PPM frame triggered the sanity check, whole frame will be droped and PPM signal values will remain
    unchanged until a valid frame can be sampled.

    PPM_error variable will be increased every time a corrupted frame was detected.
    
    RX_signalReceived flag is set to 0 every time a valid frame is processed, this variables is used to
    "tell" software there was an error in communication (anything from a corrupted frame to a complete 
    receiver failure), this flag is increased by 1 every 10ms, if it reaches 100 (represnting 1 second)
    an subroutine is triggered that should handle this failure condition, from the most simple routines
    that will just disarm the craft (making it fall from the sky like a boiled potato) to more adnvaced 
    auto-descent routines.

    Please remember that this failsafe is here to prevent craft from "flying away on its own" and
    potentionaly harming someone in the proces (damaged craft from a crash is still better then cutting
    someones head off with uncontrollable craft). 

    Big thanks to kha from #aeroquad for setting up the shared timer.
*/
#define RX_PPM_SYNCPULSE 5000 // 2.5ms

#define RX_CHANNELS 16 // dont change this
volatile uint16_t RX[RX_CHANNELS] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
volatile uint16_t PPM_temp[RX_CHANNELS] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000}; // Temporary buffer

volatile uint16_t startPulse = 0;
volatile uint8_t  ppmCounter = RX_CHANNELS;
volatile uint16_t PPM_error = 0;

volatile uint8_t RX_signalReceived = 0;

bool failsafeEnabled = false;

#define TIMER1_FREQUENCY_HZ 50
#define TIMER1_PRESCALER    8
#define TIMER1_PERIOD       (F_CPU/TIMER1_PRESCALER/TIMER1_FREQUENCY_HZ)

// Capture Interrupt Vector
ISR(TIMER1_CAPT_vect) {
    unsigned int stopPulse = ICR1;
    unsigned int pulseWidth = stopPulse - startPulse;

    // Error / Sanity check
    // if pulseWidth < 900us or pulseWidth > 2100us and pulseWidth < RX_PPM_SYNCPULSE
    if (pulseWidth < 1800 || (pulseWidth > 4200 && pulseWidth < RX_PPM_SYNCPULSE)) {
        PPM_error++;
        
        // set ppmCounter out of range so rest and (later on) whole frame is dropped
        ppmCounter = RX_CHANNELS + 1;    
    }
    
    if (pulseWidth >= RX_PPM_SYNCPULSE) {  // Verify if this is the sync pulse
        if (ppmCounter <= RX_CHANNELS) {
            // This indicates that we received an correct frame = push to the "main" PPM array
            // if we received an broken frame, it will get ignored here and later get over-written
            // by new data, that will also be checked for sanity.
            for (uint8_t i = 0; i < RX_CHANNELS; i++) {
                RX[i] = PPM_temp[i];
            }

            // Bring failsafe flag down every time we accept a valid signal / frame
            RX_signalReceived = 0;
        }
        ppmCounter = 0; // restart the channel counter
    } else {
        if (ppmCounter < RX_CHANNELS) {      // extra RX_CHANNELS will get ignored here
            PPM_temp[ppmCounter] = pulseWidth / 2; // Store measured pulse length in us
            ppmCounter++;                     // Advance to next channel
        }
    }
 
    startPulse = stopPulse; // Save time at pulse start
}

// Timer1 setup (shared timer for PPM sampling & PWM servo signal generation)
void setupTimer1RX() {
    // Setup timer1 in normal mode, count at 2MHz
    TCCR1A = 0;
    TCCR1B = (1 << CS11) | (1 << ICES1);
    TIMSK1 |= (1 << ICIE1) | (1 << OCIE1A); // Enable ICP and OCRA interrupts
}

void initializeReceiver() {
    setupTimer1RX();
}

void RX_failSafe() {
    RX_signalReceived++; // if this flag reaches 10, an auto-descent routine will be triggered.
    
    if (RX_signalReceived > 10) {
        RX_signalReceived = 10; // don't let the variable overflow
        failsafeEnabled = true; // this ensures that failsafe will operate in attitude mode
        
        // Bear in mind that this is here just to "slow" the fall, if you have lets say 500m altitude,
        // this probably won't help you much (sorry).
        // This will slowly (-2 every 100ms) bring the throttle to 1000 (still saved in the PPM array)
        // 1000 = 0 throttle;
        // Descending from FULL throttle 2000 (most unlikely) would take about 1 minute and 40 seconds
        // Descending from HALF throttle 1500 (more likely) would take about 50 seconds
        RX[CONFIG.data.CHANNEL_ASSIGNMENT[THROTTLE]] -= 2;
        
        if (RX[CONFIG.data.CHANNEL_ASSIGNMENT[THROTTLE]] < 1000) {
            RX[CONFIG.data.CHANNEL_ASSIGNMENT[THROTTLE]] = 1000; // don't let the value fall below 1000
        }    
    } else {
        failsafeEnabled = false;
    }
}