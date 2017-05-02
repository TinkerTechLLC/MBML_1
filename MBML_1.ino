/*****************************************
MBML-1 Button Control Board Firmware:

This firmware handles input from two buttons, one controlling
the main power, the other controlling the projector power. Either
button may be pressed to turn the unit on. Once the unit is on,
holding the projector button for 3 seconds will turn off only the
projector, but leave the other components on. Holding the main
power button for 3 seconds will cause the entire unit to power down.

Projector power up: 75 seconds
Projector power down 95 seconds

*****************************************/

#define DEBUG false

// Dependent libraries
#include <TimerOne.h>
#include <EEPROM.h>

// Set input / output labels
#define MAIN_BTN    4
#define ROJ_BTN     3
#define PC          5
#define FAN         6
#define SPKR_PWR    8
#define AUX         9
#define MON         10
#define SPKR_SENSE  11
#define MAIN_LED    12
#define PROJ_LED    13
#define BLANK       A7

// Component power vars
bool main_pwr   = false;
bool spkr_pwr   = false;
bool mon_pwr    = false;
bool pc_pwr     = false;
bool proj_pwr   = false;
bool fan_pwr    = false;

int load_num    = 2;   // Change this number when re-loading
                       // firmware in reset EEPROM values to defaults

int BTN_HOLD_THRESHOLD = 3000;

// EEPROM Addresses
const int EE_LOAD_NUM   = 0;
const int EE_SPRK       = 2;

long proj_on_time       = 0;
long proj_off_time      = 0;
long PROJ_STARTUP_TIME  = 75000;
long PROJ_SHUTDOWN_TIME = 95000;

long main_on_time       = 0;
long main_off_time      = 0;
long MAIN_STARTUP_TIME  = 20000;
long MAIN_SHUTDOWN_TIME = 10000;

#define OFF            -1
#define ON              0
#define STARTUP         1
#define SHUTDOWN        2
#define STARTUP_DWN     3   // In startup mode, but shutdown is scheduled

int main_state;
int proj_state;

#define RELEASED        0
#define TAPPED          1
#define HELD            2

int main_btn_state;
int proj_btn_state;

const int ISR_DELAY = 50000;

// Projector commands
const char* PROJ_CMD_ON = "\r*pow=on#\r";
const char* PROJ_CMD_OFF = "\r*pow=off#\r";

void setup() {

    Serial.begin(115200);   // Baud rate determined by projector interface

    // Set i/o states
    pinMode(MAIN_BTN, INPUT);
    pinMode(PROJ_BTN, INPUT);
    pinMode(PC, OUTPUT);
    pinMode(FAN, OUTPUT);
    pinMode(SPKR_PWR, OUTPUT);
    pinMode(AUX, OUTPUT);
    pinMode(MON, OUTPUT);
    pinMode(SPKR_SENSE, INPUT);
    pinMode(MAIN_LED, OUTPUT);
    pinMode(PROJ_LED, OUTPUT);

    // Set all output pins off
    digitalWrite(PC, LOW);
    digitalWrite(FAN, LOW);
    digitalWrite(SPKR_PWR, LOW);
    digitalWrite(AUX, LOW);
    digitalWrite(MON, LOW);
    digitalWrite(MAIN_LED, LOW);
    digitalWrite(PROJ_LED, LOW);

    // Setup the timer for LED blinking
    Timer1.initialize();
    Timer1.setPeriod(ISR_DELAY);
    Timer1.attachInterrupt(LEDCheck);

    // Check the phototransistor to see if the speaker's power LED is on
    // so the spkr_pwr variable is in an accurate initial state
    spkr_pwr = digitalRead(SPKR_SENSE);

    projState(OFF);
    mainState(OFF);
    main_btn_state = RELEASED;
    proj_btn_state = RELEASED;
}

void loop() {
    // Monitor the button states
    checkBtn(MAIN_BTN);
    checkBtn(PROJ_BTN);
    delay(10);
    handleButtons();
    handleStates();
}

void handleButtons(){
    if(main_btn_state == TAPPED && mainState() == OFF){
        mainState(STARTUP);
        setMainPwr(true);
    }
    else if(main_btn_state == HELD && mainState() == ON){
        if(projState() == ON){
            projState(SHUTDOWN);
            setProjPwr(false);
        }
        mainState(SHUTDOWN);
        setMainPwr(false);
        // If the projector is still warming up, schedule a shutdown for later
        if(projState() == STARTUP){
            projState(STARTUP_DWN);
        }
    }
    else if(proj_btn_state == TAPPED && projState() == OFF){
        if(mainState() == OFF){
            mainState(STARTUP);
            setMainPwr(true);
        }
        projState(STARTUP);
        setProjPwr(true);
        setFanPwr(true);
    }
    else if(proj_btn_state == HELD){
        if(projState() == ON){
            projState(SHUTDOWN);
            setProjPwr(false);
        }
        else if(projState() == STARTUP){
            projState(STARTUP_DWN);
        }
    }
}

void handleStates(){
    // Handle main states
    if(mainState() == STARTUP && millis() - main_on_time > MAIN_STARTUP_TIME){
        mainState(ON);
    }
    else if(mainState() == SHUTDOWN && millis() - main_off_time > MAIN_SHUTDOWN_TIME){
        mainState(OFF);
    }

    // Handle projector States
    if(projState() == STARTUP_DWN && millis() - proj_on_time > PROJ_STARTUP_TIME){
        projState(SHUTDOWN);
        setProjPwr(false);

    }
    else if(projState() == STARTUP && millis() - proj_on_time > PROJ_STARTUP_TIME){
        projState(ON);
    }
    else if(projState() == SHUTDOWN && millis() - proj_off_time > PROJ_SHUTDOWN_TIME){
        if(DEBUG){
            Serial.println("Transitioning from SHUTDOWN to OFF");
            Serial.print("Cur time: ");
            Serial.print(millis());
            Serial.print(" proj_off_time: ");
            Serial.print(proj_off_time);
            Serial.print(" elapsed shutdown time: ");
            Serial.println(millis() - proj_off_time);
        }
        setFanPwr(false);
        projState(OFF);
    }
}

void checkBtn(int btn_pin){

    static long main_start = 0;
    static long proj_start = 0;
    static bool main_last_pressed = false;
    static bool proj_last_pressed = false;
    const int DEBOUNCE = 50;
    static long main_press_time = -1;
    static long proj_press_time = -1;

    long* btn_start;
    bool* btn_last_pressed;
    int* btn_state;
    long* btn_press_time;

    if(btn_pin == MAIN_BTN){
        btn_start = &main_start;
        btn_last_pressed = &main_last_pressed;
        btn_state = &main_btn_state;
        btn_press_time = &main_press_time;
    }
    else if(btn_pin == PROJ_BTN){
        btn_start = &proj_start;
        btn_last_pressed = &proj_last_pressed;
        btn_state = &proj_btn_state;
        btn_press_time = &proj_press_time;
    }
    else{
        if(DEBUG){
            Serial.println("Invalid button check request");
        }
        return;
    }

    bool cur_press = !digitalRead(btn_pin);
    // Initial press
    if(cur_press && !(*btn_last_pressed)){
        *btn_start = millis();
        *btn_last_pressed = true;
        if(DEBUG){
            Serial.println("Setting initial press");
        }
        *btn_press_time = millis();
    }
    else if(cur_press && *btn_last_pressed){
        if(millis() - *btn_press_time > BTN_HOLD_THRESHOLD){
            if(DEBUG){
                Serial.print("Button held: ");
                Serial.println(btn_pin);
            }
            *btn_state = HELD;
        }
    }
    else if(!cur_press){
        if(*btn_last_pressed && *btn_state != HELD){
            if(DEBUG){
                Serial.print("Button tapped: ");
                Serial.println(btn_pin);
            }
            *btn_state = TAPPED;
        }
        // Persistent release
        else{
            *btn_state = RELEASED;
        }
        *btn_last_pressed = false;
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
            tapButton(PC);
        }
    }
}

void setFanPwr(bool pwr_state){
    fan_pwr = pwr_state;
    if(fan_pwr){
        digitalWrite(FAN, HIGH);
    }
    else{
        digitalWrite(FAN, LOW);
    }
}

void setProjPwr(bool pwr_state){

    proj_pwr = pwr_state;
    // Projector startup sequence:
    if(proj_pwr){
        proj_on_time = millis();
        // Tap button once
        Serial.print(PROJ_CMD_ON);
    }
    // Projector shutdown sequence:
    else{
        proj_off_time = millis();
        if(DEBUG){
            Serial.print("Setting proj off time: ");
            Serial.println(proj_off_time);
        }
        Serial.print(PROJ_CMD_OFF);
    }
}

void setMainPwr(bool pwr_state){
    if(main_pwr == pwr_state){
        return;
    }
    else{
        main_pwr = pwr_state;
        if(main_pwr){
            main_on_time = millis();
        }
        else{
            main_off_time = millis();
        }
        setSpkrPwr(pwr_state);
        setPCPwr(pwr_state);
        wait(5000);
        setMonPwr(pwr_state);
    }
}

/*
*   The interrupt service routine runs every period of the
*   Timer1 interrupt. This will cause any enabled LEDs
*   to blink with a frequency of 1000 / (2*delay_time)
*/
void LEDCheck(){
    int proj_delay = 500;
    int main_delay = 500;
    static long main_cycle_start = 0;
    static long proj_cycle_start = 0;
    static bool main_blink_state = false;
    static bool proj_blink_state = false;
    static int old_main = OFF;
    static int old_proj = OFF;

    // Set main LED blink rate
    switch(mainState()){
      case STARTUP:
        main_delay = 500;
      break;
      case SHUTDOWN:
        main_delay = 500;
      break;
    }

    // Set projector LED blink rate
    switch(projState()){
      case STARTUP:
        proj_delay = 500;
      break;
      case SHUTDOWN:
        proj_delay = 500;
      break;
      case STARTUP_DWN:
        proj_delay = 200;
    }

    // Only write constant on/off if state has changed so we
    // don't waste cyles writing it every time the ISR runs
    if(mainState() != old_main){
        if(mainState() == OFF){
            digitalWrite(MAIN_LED, LOW);
        }
        else if(mainState() == ON){
            digitalWrite(MAIN_LED, HIGH);
        }
    }
    else if(mainState() != OFF
            && mainState() != ON
            && millis() - main_cycle_start > main_delay){
      digitalWrite(MAIN_LED, main_blink_state);
      main_cycle_start = millis();
      main_blink_state = !main_blink_state;
    }

    if(projState() != old_proj){
        if(projState() == OFF){
            digitalWrite(PROJ_LED, LOW);
        }
        else if(projState() == ON){
            digitalWrite(PROJ_LED, HIGH);
        }
    }
    else if(projState() != OFF
        && projState() != ON
        && millis() - proj_cycle_start > proj_delay){
      digitalWrite(PROJ_LED, proj_blink_state);
      proj_cycle_start = millis();
      proj_blink_state = !proj_blink_state;
    }

    old_main = mainState();
    old_proj = projState();
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
        tapButton(SPKR_PWR);
    }
}

void tapButton(int pin){
    digitalWrite(pin, HIGH);
    delay(200);
    digitalWrite(pin, LOW);
}

/**
 * Causes the program to idle without interfering with interrupts.
 *
 * @param time_ms How long to idle the program in milliseconds
 */
void wait(int time_ms){
    long start_time = millis();
    while(millis() - start_time < time_ms){
        // Don't do anything, but don't use
        // delay() so we don't get in the way
        // the interrupt timer.
    }
}

void mainState(int state){
    main_state = state;
    if(DEBUG){
        Serial.print("Main state: ");
    }
    printState(state);
}

int mainState(){
    return main_state;
}

void projState(int state){
    proj_state = state;
    if(DEBUG){
        Serial.print("Proj state: ");
    }
    printState(state);
}

int projState(){
    return proj_state;
}

void printState(int state){
    if(!DEBUG){
        return;
    }
    switch (state) {
        case OFF:
        Serial.println("OFF");
        break;
        case STARTUP:
        Serial.println("STARTUP");
        break;
        case ON:
        Serial.println("ON");
        break;
        case SHUTDOWN:
        Serial.println("SHUTDOWN");
        break;
        case STARTUP_DWN:
        Serial.println("STARTUP_DWN");
        break;
    }
}
