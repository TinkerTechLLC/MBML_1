/*****************************************
MBML-1 Button Control Board Firmware:

This firmware handles input from two buttons, one controlling
the main power, the other controlling the projector power. Either
button may be pressed to turn the unit on. Once the unit is on,
holding the projector button for 3 seconds will turn off only the
projector, but leave the other components on. Holding the main
power button for 3 seconds will cause the entire unit to power down.

*****************************************/

// Dependent libraries
#include <TimerOne.h>
#include <EEPROM.h>

// Set input / output labels
const int PWR_BTN = 3;
const int PROJ_BTN = 4;
const int PC = 5;
const int PROJ = 6;
const int SPKR = 8;
const int AUX = 9;
const int MON = 10;
const int MAIN_LED = 11;
const int PROJ_LED = 12;

// Component power vars
bool main_pwr = false;
bool spkr_pwr = false;
bool mon_pwr = false;
bool pc_pwr = false;
bool proj_pwr = false;

bool main_LED_blink = false;
bool proj_LED_blink = false;

int load_num = 0;   // Change this number when re-loading
                    // firmware in reset EEPROM values to defaults

int power_off_wait = 3000;

// EEPROM Addresses
const int EE_LOAD_NUM = 0;
const int EE_SPRK = 2;

void setup() {
  Serial.begin(9600);
    // Set i/o states
    pinMode(PWR_BTN, INPUT);
    pinMode(PROJ_BTN, INPUT);
    pinMode(PC, OUTPUT);
    pinMode(PROJ, OUTPUT);
    pinMode(SPKR, OUTPUT);
    pinMode(AUX, OUTPUT);
    pinMode(MON, OUTPUT);

    // Setup the timer for LED blinking
    Timer1.initialize();
    Timer1.setPeriod(50000);
    Timer1.attachInterrupt(blinkCheck);

    // If the firmware has just been loaded, save the default speaker state
    if(load_num != EEPROM.read(EE_LOAD_NUM)){
        EEPROM.write(load_num, EE_LOAD_NUM);
        EEPROM.write(false, EE_SPRK);
    }
    // If we haven't, then load the speaker state
    else{
        spkr_pwr = EEPROM.read(EE_SPRK);
    }
}

void loop() {
    // Monitor the button states
    checkMainBtn();
    checkProjBtn();
    delay(10);
}

void checkMainBtn(){
    static long main_btn_start = 0;
    static bool main_is_pressed = false;
    // Read the main button
    if(digitalRead(PWR_BTN)){
        if(!main_is_pressed){
            main_is_pressed = true;
            main_btn_start = millis();
            Serial.println("Power pressed");
        }
        // Turn on immediately
        if(!main_pwr){
            setMainPwr(true);
        }
        else{
            // Don't shutdown until the button has been held long enough
            if(millis() - main_btn_start > power_off_wait){
                setMainPwr(false);
            }
        }
    }
    else{
        main_is_pressed = false;
    }
}

void checkProjBtn(){
    static long proj_btn_start = 0;
    static bool proj_is_pressed = false;

    // Read the projector button state
    if(digitalRead(PROJ_BTN)){
        if(!proj_is_pressed){
            proj_is_pressed = true;
            proj_btn_start = millis();
            Serial.println("Proj pressed");
        }
        // If the unit is off, start it
        if(!main_pwr){
            setMainPwr(true);
        }
        else{
            // Immediately turn on the projector if it is off
            if(!proj_pwr){
                setProjPwr(true);
            }
            else{
                // If the button has been pressed long enough, turn off the projector
                if(millis() - proj_btn_start > power_off_wait){
                    setProjPwr(false);
                }
            }
        }
    }
    else{
        proj_is_pressed = false;
    }
}

void setPCPwr(bool pwr_state){
    // Don't do anything unless chaning state
    if(pc_pwr == pwr_state){
        return;
    }
    else{
        pc_pwr = pwr_state;
        // Projector startup sequence:
        if(pc_pwr){
            // Tap button once
            tapButton(PC);
        }
        // Projector shutdown sequence:
        else{
            // Long press
            holdButton(PC, 4000);
        }
    }
}

void setProjPwr(bool pwr_state){
    // Don't do anything unless chaning state
    if(proj_pwr == pwr_state){
        return;
    }
    else{
        proj_pwr = pwr_state;
        // Projector startup sequence:
        if(proj_pwr){
            // Tap button once
            tapButton(PROJ);
            // Wait a few seconds
            projBlinkWait(2000);
            // Tap the power button again to confirm
            tapButton(PROJ);
        }
        // Projector shutdown sequence:
        else{
            // Tap button once
            tapButton(PROJ);
            // Wait a few seconds
            projBlinkWait(4000);
            // Tap the power button again to confirm
            tapButton(PROJ);
        }
    }
}


/*
*   The interrupt service routine runs every period of the
*   Timer1 interrupt. This will cause the any enabled LEDs
*   to blink with a frequency of 1000 / (2*delay_time)
*/
void blinkCheck(){
    int delay_time = 100;
    static long cycle_start = 0;
    static bool blink_state = false;

    if(millis() - cycle_start > delay_time){
        blink_state = !blink_state;
        if(main_LED_blink){
            digitalWrite(MAIN_LED, blink_state);
        }
        if(proj_LED_blink){
            digitalWrite(PROJ_LED, blink_state);
        }
        cycle_start = millis();
    }
}

void setMonPwr(bool pwr_state){
    if(mon_pwr == pwr_state){
        return;
    }
    else{
        mon_pwr = pwr_state;
        digitalWrite(MON, pwr_state);
    }
}

void setSpkrPwr(bool pwr_state){
    if(spkr_pwr == pwr_state){
        return;
    }
    else{
        spkr_pwr = pwr_state;
        tapButton(SPKR);
        // Save the speaker state to restore after power cycle
        EEPROM.write(spkr_pwr, EE_SPRK);
    }
}

void setMainPwr(bool pwr_state){
    if(main_pwr == pwr_state){
        return;
    }
    else{
        main_pwr = pwr_state;
        mainBlink(true);
        setMonPwr(pwr_state);
        setSpkrPwr(pwr_state);
        setPCPwr(pwr_state);
        setProjPwr(pwr_state);
        mainBlink(false);
    }
}

void tapButton(int pin){
    digitalWrite(pin, HIGH);
    delay(100);
    digitalWrite(pin, LOW);
}

void holdButton(int pin, int wait){
  digitalWrite(pin, HIGH);
  delay(wait);
  digitalWrite(pin, LOW);
}

void wait(int time_ms){
    long start_time = millis();
    while(millis() - start_time < time_ms){
        // Don't do anything, but don't use
        // delay() so we don't get in the way
        // the interrupt timer.
    }
}

void mainBlink(bool is_blink){
  main_LED_blink = is_blink;
}

void projBlinkWait(int time_ms){
    proj_LED_blink = true;
    wait(time_ms);
    proj_LED_blink = false;
}
