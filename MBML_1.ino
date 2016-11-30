/*****************************************
MBML-1 Button Control Board Firmware:

This firmware handles input from two buttons, one controlling
the main power, the other controlling the projector power. Either
button may be pressed to turn the unit on. Once the unit is on,
holding the projector button for 3 seconds will turn off only the
projector, but leave the other components on. Holding the main
power button for 3 seconds will cause the entire unit to power down.

Projector power up: 75 seconds
Projector power down 90 seconds

*****************************************/

// Dependent libraries
#include <TimerOne.h>
#include <EEPROM.h>

// Set input / output labels
const int MAIN_BTN = 3;
const int PROJ_BTN = 4;
const int PC = 5;
const int PROJ = 6;
const int SPKR = 8;
const int AUX = 9;
const int MON = 10;
const int MAIN_LED = 12;
const int PROJ_LED = 11;

// Component power vars
bool main_pwr = false;
bool spkr_pwr = false;
bool mon_pwr = false;
bool pc_pwr = false;
bool proj_pwr = false;

int load_num = 1;   // Change this number when re-loading
                    // firmware in reset EEPROM values to defaults

int BTN_HOLD_THRESHOLD = 3000;

// EEPROM Addresses
const int EE_LOAD_NUM = 0;
const int EE_SPRK = 2;

long proj_on_time = 0;
long proj_off_time = 0;
long PROJ_STARTUP_TIME = 75000;
long PROJ_SHUTDOWN_TIME = 95000;

long main_on_time = 0;
long main_off_time = 0;
long MAIN_STARTUP_TIME = 20000;
long MAIN_SHUTDOWN_TIME = 10000;

#define OFF -1
#define ON 0
#define STARTUP 1
#define SHUTDOWN 2
#define STARTUP_DWN 3   // In startup mode, but shutdown is scheduled

int main_state;
int proj_state;

#define RELEASED 0
#define TAPPED 1
#define HELD 2

int main_btn_state;
int proj_btn_state;

const int ISR_DELAY = 50000;

void setup() {
    Serial.begin(9600);
    // Set i/o states
    pinMode(MAIN_BTN, INPUT);
    pinMode(PROJ_BTN, INPUT);
    pinMode(PC, OUTPUT);
    pinMode(PROJ, OUTPUT);
    pinMode(SPKR, OUTPUT);
    pinMode(AUX, OUTPUT);
    pinMode(MON, OUTPUT);

    // Setup the timer for LED blinking
    Timer1.initialize();
    Timer1.setPeriod(ISR_DELAY);
    Timer1.attachInterrupt(LEDCheck);

    // If the firmware has just been loaded, save the default speaker state
    if(load_num != EEPROM.read(EE_LOAD_NUM)){
        EEPROM.write(load_num, EE_LOAD_NUM);
        EEPROM.write(false, EE_SPRK);
    }
    // If we haven't, then load the speaker state
    else{
        spkr_pwr = EEPROM.read(EE_SPRK);
    }

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
        Serial.println("Transitioning from SHUTDOWN to OFF");
        Serial.print("Cur time: ");
        Serial.print(millis());
        Serial.print(" proj_off_time: ");
        Serial.print(proj_off_time);
        Serial.print(" elapsed shutdown time: ");
        Serial.println(millis() - proj_off_time);
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
        Serial.println("Invalid button check request");
        return;
    }

    bool cur_press = !digitalRead(btn_pin);
    // Initial press
    if(cur_press && !(*btn_last_pressed)){
        *btn_start = millis();
        *btn_last_pressed = true;
        Serial.println("Setting initial press");
        *btn_press_time = millis();
    }
    else if(cur_press && *btn_last_pressed){
        if(millis() - *btn_press_time > BTN_HOLD_THRESHOLD){
            Serial.print("Button held: ");
            Serial.println(btn_pin);
            *btn_state = HELD;
        }
    }
    else if(!cur_press){
        if(*btn_last_pressed && *btn_state != HELD){
            Serial.print("Button tapped: ");
            Serial.println(btn_pin);
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

void setProjPwr(bool pwr_state){

    proj_pwr = pwr_state;
    // Projector startup sequence:
    if(proj_pwr){
        proj_on_time = millis();
        // Tap button once
        tapButton(PROJ);
    }
    // Projector shutdown sequence:
    else{
        proj_off_time = millis();
        Serial.print("Setting proj off time: ");
        Serial.println(proj_off_time);
        // Tap button once
        tapButton(PROJ);
        // Wait a second
        wait(1000);
        // Tap the power button again to confirm
        tapButton(PROJ);
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
*   Timer1 interrupt. This will cause the any enabled LEDs
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
        tapButton(SPKR);
        // Save the speaker state to restore after power cycle
        EEPROM.write(spkr_pwr, EE_SPRK);
        // Verify EEPROM
        bool test = EEPROM.read(EE_SPRK);
        Serial.print("EEPROM state: ");
        Serial.println(test);
    }
}

void tapButton(int pin){
    digitalWrite(pin, HIGH);
    delay(200);
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

void mainState(int state){
    main_state = state;
    Serial.print("Main state: ");
    printState(state);
}

int mainState(){
    return main_state;
}

void projState(int state){
    proj_state = state;
    Serial.print("Proj state: ");
    printState(state);
}

int projState(){
    return proj_state;
}

void printState(int state){
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
