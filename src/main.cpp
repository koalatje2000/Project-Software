/**********************************************************************
 * Project Software                                                   *
 * Wouter Denkers & Daan Ypma (ETIV.1A)                               *
 * Datum 12 January 2021                                              *
 * Version: 2.4.1 (Last update: 12-January-2021)                      *
 **********************************************************************/


/**********************************************************************
 *                          Used Library's                            *
 **********************************************************************/
#include "../lib/Servo/src/Servo.h"
#include <Arduino.h>
#include <EEPROM.h>

/**********************************************************************
 *                          Pinnen declaration                        *
 **********************************************************************/
#define BCDDecoder_PinA 2
#define BCDDecoder_PinB 3
#define BCDDecoder_PinC 4
#define BCDDecoder_PinD 5
#define doorclose_switch_pin 6
#define shiftRegisterPinC 7  //DP
#define DIAL_ENCODER_INPUT_PINA 8
#define DIAL_ENCODER_INPUT_PINB 9
#define SERVO_PWM_PIN 10
Servo myservo;
#define shiftRegisterPinA 11 //LP
#define DIAL_BUTTON_INPUT_PIN 12
#define shiftRegisterPinB 13 //CP
#define Led1Pin1 A0
#define Led1Pin2 A1
#define PirSensorPin A2
#define PirEnableButton A3
#define ResetCodeButton A4
#define BUZZER_PIN A5

/**********************************************************************
 *                          Function declaration                      *
 **********************************************************************/
int getValue_Dial_Knob_Encoder();
void setPin_Dial_Knob_Button();
void setPin_Dial_Knob_Encoder();
int getValue_Dial_Button(int WaitUntilSwitchOff);
int getCode(int NewCode);
int getValue_Dial_Knob_Encoder(int Min, int Max, int InfinityTurn);
int validateCode();
void resetCode();
void set_DoorCloseSensor();
int get_DoorCloseSensor();
void ShowFilledInCode(int insertCode[], int MaxCodePosition, int CurrentPositionNumber);
void RunInTimer(int CurrentFilledInCode[], int MaxCodePosition, int CurrentPositionNumber);
void move_Servo(int open);
void set_Servo();
void setBuzzerOn(int Aan);
void setBuzzer();
void setBCDDecoder();
void setShifter();
void checkPirButton();
void usePirSensor();
void getValuesEEPROMP();
void setValuesEEPROMP();
void checkDoorCloseButton();
void statesHandling();
void setPirsensorPins();

/**********************************************************************
 *                          State Variables                           *
 **********************************************************************/
int doorState = 0;  //1 open 0 closed
int changeCode = 0; //1 open 0 closed
int pirSensorActivityDetected = 0; // 1 active | 0 not active
int pirEnabled = 0; //1 is enabled | 0 is not enabled

/**********************************************************************
 *                      Delay Timing Variables                        *
 **********************************************************************/
int TimerDisplay = 0;
int RunInTimerDelay = 0;
unsigned long delayTimer2 = NULL;
unsigned long delayTimer = millis();

/**********************************************************************
 *                        Changeble Variables                         *
 **********************************************************************/
const int MaxCodePosition = 4;
int CurrentFilledInCode[MaxCodePosition] = {0,0,0,0};
int CurrentCorrectCode[MaxCodePosition] = {4,4,4,4};
byte x = 0b10000000;
int delayNumber = 7;

/**********************************************************************
 *                         Pin State Variables                        *
 *      [Could not be global but because of time problems they are]   *
 **********************************************************************/
int aState;
int aLastState;
int LastValueDailButton = 0;
int btn_prev = LOW;

/**********************************************************************
 *                          Encoder Variables                         *
 *      [Could not be global but because of time problems they are]   *
 **********************************************************************/
double dial_knob_encoder_value = 0;
int buttonPushedCounter = 0;
int CurrentPositionNumber = 0;
int DisplayPosition = 0;

/******************************************************************************
*                                                                             *
*   Function : void setup()                                                   *
*                                                                             *
*   Parameters : none                                                         *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will run 1 time and always on the start of  *
*                 the program.                                                *
*                                                                             *
*******************************************************************************/
void setup() {
    getValuesEEPROMP();
    Serial.begin(9600); //debugg line
    setPin_Dial_Knob_Button();
    setPin_Dial_Knob_Encoder();
    set_DoorCloseSensor();
    set_Servo();
    setBuzzer();
    setShifter();
    setBCDDecoder();
    setPirsensorPins();
}

/******************************************************************************
*                                                                             *
*   Function : void setShifter()                                              *
*                                                                             *
*   Parameters : none                                                         *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will setup all pins for the Shift-Register  *
*                 so they defined as outputs.                                 *
*                                                                             *
*******************************************************************************/
void setShifter(){
    pinMode(shiftRegisterPinA, OUTPUT);
    pinMode(shiftRegisterPinC, OUTPUT);
    pinMode(shiftRegisterPinB, OUTPUT);
}

/******************************************************************************
*                                                                             *
*   Function : void setBCDDecoder()                                           *
*                                                                             *
*   Parameters : none                                                         *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will setup all pins for the BCD-Decoder     *
*                 so they defined as outputs.                                 *
*                                                                             *
*******************************************************************************/
void setBCDDecoder(){
    pinMode(BCDDecoder_PinA, OUTPUT);
    pinMode(BCDDecoder_PinB, OUTPUT);
    pinMode(BCDDecoder_PinC, OUTPUT);
    pinMode(BCDDecoder_PinD, OUTPUT);
}

/******************************************************************************
*                                                                             *
*   Function : void ResetCodeBTN()                                            *
*                                                                             *
*   Parameters : none                                                         *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will check if the reset filled in code      *
*                 button is pressed.                                          *
*                                                                             *
*******************************************************************************/
void ResetCodeBTN(){
    if(digitalRead(ResetCodeButton) == HIGH) {
        resetCode();
    }
}

/******************************************************************************
*                                                                             *
*   Function : void setValuesEEPROMP()                                        *
*                                                                             *
*   Parameters : none                                                         *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will read all values from the correct code  *
*                 array and put them in the EEPROMP of the Arduino UNO.       *
*                                                                             *
*******************************************************************************/
void setValuesEEPROMP(){
    for (int i = 0; i <= MaxCodePosition; i++) {
        EEPROM.write(i, CurrentCorrectCode[i]);
    }
}

/******************************************************************************
*                                                                             *
*   Function : void getValuesEEPROMP()                                        *
*                                                                             *
*   Parameters : none                                                         *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will read all values from the EEPROMP of    *
*                 the Arduino UNO and put in the correct code array.          *
*                                                                             *
*******************************************************************************/
void getValuesEEPROMP(){
    for (int i = 0; i <= MaxCodePosition; i++) {
        int value = EEPROM.read(i); //Code from EEPROMP to array
        CurrentCorrectCode[i] = value;
    }
}

/******************************************************************************
*                                                                             *
*   Function : void loop()                                                    *
*                                                                             *
*   Parameters : none                                                         *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will continues run when the Arduino UNO     *
*                 has power after the setup() function.                       *
*                                                                             *
*******************************************************************************/
void loop() {
    ResetCodeBTN();         //Will check if the Reset Code Button is Pressed.
    checkPirButton();       //Will check if the afstandsensor is active.
    statesHandling();       //Will Check the state and activate different steps.
    usePirSensor();         //Will check pir sensor
}

/******************************************************************************
*                                                                             *
*   Function : void statesHandling()                                          *
*                                                                             *
*   Parameters : none                                                         *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will check the current state and looks what *
*                 the code should run next or deal with changing the          *
*                 different states.                                           *
*                                                                             *
*******************************************************************************/
void statesHandling(){
    if (doorState) {        //Will check in what state the kluis is OPEN or CLOSED
        delayTimer2 = NULL; //Reset the Pir sensor time

        // Will check if the Dial-Knop is pressed and not is in Change Code state
        if (getValue_Dial_Button(0) && !changeCode) {
            Serial.println(buttonPushedCounter); //debug line
            buttonPushedCounter++;
            int WaitingArray[MaxCodePosition] = {0,0,0,0};
            RunInTimer(WaitingArray, MaxCodePosition, CurrentPositionNumber); //Will display the current filled in code

            //Will check if the button is pressed longer then 5 sec
            if (buttonPushedCounter > 1000) { // NOTE: This loop runs 5 times a milisecond
                changeCode = 1; //Change state to changeCode state
                buttonPushedCounter = 0; //Reset ButtonPressed to 0;
            }
            resetCode(); //reset display
        } else { // if the Dial-Button isn't pressed or in the Change Code state
            buttonPushedCounter = 0;
            if(getCode(changeCode)) { //Change the code of the safe so it is the new one that is filled in
                Serial.println("Code Changed Of Safe"); //debug line
                resetCode(); //reset display
                changeCode = 0; //set state back
            }
        }
        checkDoorCloseButton(); //Will check if the door switch is pressed
    }
    else if(getCode(changeCode)) {
        if (validateCode()) { //will validate the code
            Serial.println("You are in the safe!!"); //debug line
            move_Servo(1); //Will open the door
            doorState = 1; //change the door state

        } else {
            Serial.println("WRONG CODE"); //debug line
            setBuzzerOn(1); //set alarm on
            delay(20); //small delay
            setBuzzerOn(0); //set alarm off
        }
        resetCode(); //reset display
    }
}

/******************************************************************************
*                                                                             *
*   Function : void checkDoorCloseButton()                                    *
*                                                                             *
*   Parameters : none                                                         *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will run different steps when the door      *
*                 close button is pressed.                                    *
*                                                                             *
*******************************************************************************/
void checkDoorCloseButton(){
    if (get_DoorCloseSensor()){
        move_Servo(0);
        doorState = 0;
        Serial.println("CLOSING DOOR!"); //debug line
        resetCode();
    }
}

/******************************************************************************
*                                                                             *
*   Function : int getCode(int NewCode)                                       *
*                                                                             *
*   Parameters : int NewCode - Is a parameter with indicate the change        *
*                code state or not.                                           *
*                                                                             *
*   Return type : will return if the number is correct filled in.             *
*                                                                             *
*   Description : A function that will run different steps depending on the   *
*                state.                                                       *
*                                                                             *
*******************************************************************************/
int getCode(int NewCode) {
    if (!NewCode){ //Checks if it is a new code or not
        int CodeFilledIn = 0;
        CurrentFilledInCode[CurrentPositionNumber] = getValue_Dial_Knob_Encoder(0, 9, 0);
        if (getValue_Dial_Button(1)) {
            if (((millis() - delayTimer) >= 200)) { //Small delay to smooth the signal
                delayTimer = millis();
                if (CurrentPositionNumber == MaxCodePosition - 1) {
                    CodeFilledIn = 1;
                } else {
                    RunInTimer(CurrentFilledInCode, MaxCodePosition, CurrentPositionNumber); //Display numbers in a sync delay
                }
                CurrentPositionNumber++;
            }
        } else {
            RunInTimer(CurrentFilledInCode, MaxCodePosition, CurrentPositionNumber); //Display numbers in a sync delay
        }
        return CodeFilledIn;
    } else {
        int CodeFilledIn = 0;
        CurrentCorrectCode[CurrentPositionNumber] = getValue_Dial_Knob_Encoder(0, 9, 0);
        if (getValue_Dial_Button(1)) {
            if (((millis() - delayTimer) >= 200)) { //Small delay to smooth the signal
                delayTimer = millis();
                if (CurrentPositionNumber == MaxCodePosition - 1) {
                    CodeFilledIn = 1;
                    setValuesEEPROMP(); //Will set the new code in the EEPROMP
                    changeCode = 0;
                } else {
                    RunInTimer(CurrentCorrectCode, MaxCodePosition, CurrentPositionNumber); //Display numbers in a sync delay
                }
                CurrentPositionNumber++;
            }
        } else {
            RunInTimer(CurrentCorrectCode, MaxCodePosition, CurrentPositionNumber); //Display numbers in a sync delay
        }
        return CodeFilledIn;
    }
}

/******************************************************************************
*                                                                             *
*   Function : int validateCode()                                             *
*                                                                             *
*   Parameters : none                                                         *
*                                                                             *
*   Return type : will return if the code is validated.                       *
*                                                                             *
*   Description : A function that will check if the filled in code is         *
*                 correct filled in.                                          *
*                                                                             *
*******************************************************************************/
int validateCode(){
    for (int i = 0; i < MaxCodePosition; ++i) {
        if (CurrentFilledInCode[i] != CurrentCorrectCode[i]) {
            return 0; //Not Correct
        }
    }
    return 1; //Correct
}

/******************************************************************************
*                                                                             *
*   Function : void resetCode()                                               *
*                                                                             *
*   Parameters : none                                                         *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will clear the display to 0000.             *
*                                                                             *
*******************************************************************************/
void resetCode(){
    dial_knob_encoder_value = dial_knob_encoder_value - ((int) dial_knob_encoder_value);
    for (int & i : CurrentFilledInCode) {
        i = 0;
    }
    CurrentPositionNumber = 0;
}

/******************************************************************************
*                                                                             *
*   Function :  void RunInTimer(int CurrentFilledInCode[],                    *
*                    int MaxCodePosition, int CurrentPositionNumber)          *
*                                                                             *
*   Parameters : int CurrentFilledInCode[] - An array of  the current filled  *
*                                            in code of the safe.             *
*                int MaxCodePosition - The max possible code positions.       *
*                int CurrentPositionNumber - The current position number      *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will run in a synced delay and display the  *
*                 the current filled in code.                                 *
*                                                                             *
*******************************************************************************/
void RunInTimer(int CurrentFilledInCode[], int MaxCodePosition, int CurrentPositionNumber) {
    RunInTimerDelay++;
    if (RunInTimerDelay == 15){ //20
        ShowFilledInCode(CurrentFilledInCode, MaxCodePosition, CurrentPositionNumber);
        RunInTimerDelay = 0;
    }
}

/******************************************************************************
*                                                                             *
*   Function : void updateShiftRegister()                                     *
*                                                                             *
*   Parameters : none                                                         *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will update the shift-register with the     *
*                 new display bit to show value.                              *
*                                                                             *
*******************************************************************************/
void updateShiftRegister()
{
    digitalWrite(shiftRegisterPinA, LOW); //Enable the shift-register chip
    shiftOut(shiftRegisterPinC, shiftRegisterPinB, LSBFIRST, x); //Send data to the shift-register chip
    digitalWrite(shiftRegisterPinA, HIGH); //Disable the shift-register chip
}

/******************************************************************************
*                                                                             *
*   Function : void SetNumberToBinary(int Number)                             *
*                                                                             *
*   Parameters : int Number - an int number                                   *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will set an int value to a binary one.      *
*                 And set  the data pins for the number to be displayed.      *
*                                                                             *
*******************************************************************************/
void SetNumberToBinary(int Number) {
    uint8_t bitsCount = sizeof( Number ) * 8;
    char str[ bitsCount + 1 ];
    uint8_t i = 0;
    while ( bitsCount-- ) {
        str[ i++ ] = bitRead( Number, bitsCount ) + '0';
    }
    str[ i ] = '\0';

    //set datapins
    {
        if (str[12] != str[1]) {
            digitalWrite(5, HIGH);
        } else {
            digitalWrite(5, LOW);
        }
        if (str[13] != str[1]) {
            digitalWrite(4, HIGH);
        } else {
            digitalWrite(4, LOW);
        }
        if (str[14] != str[1]) {
            digitalWrite(3, HIGH);
        } else {
            digitalWrite(3, LOW);
        }
        if (str[15] != str[1]) {
            digitalWrite(2, HIGH);
        } else {
            digitalWrite(2, LOW);
        }
    }
}

/******************************************************************************
*                                                                             *
*   Function : void SetBCDDecoder(int Position)                               *
*                                                                             *
*   Parameters : int Position - an int position number to display             *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will set all displays of and the one that   *
*                 need to display a number on.                                *
*                                                                             *
*******************************************************************************/
void SetBCDDecoder(int Position) {
    updateShiftRegister();
    x = 0b10000000;
    bitSet(x, 4 - Position);
    updateShiftRegister();
}

/******************************************************************************
*                                                                             *
*   Function :  void ShowFilledInCode(int insertCode[],                       *
*                    int MaxCodePosition, int CurrentPositionNumber)          *
*                                                                             *
*   Parameters : int insertCode[] - An array of  the current filled           *
*                                   in code of the safe.                      *
*                int MaxCodePosition - The max possible code positions.       *
*                int CurrentPositionNumber - The current position number      *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will run in a delay and display the         *
*                 the current filled in code.                                 *
*                                                                             *
*******************************************************************************/
void ShowFilledInCode(int insertCode[], int MaxCodePosition, int CurrentPositionNumber) {
    if (TimerDisplay == delayNumber){
        SetNumberToBinary(insertCode[DisplayPosition]);
        SetBCDDecoder(DisplayPosition);
        TimerDisplay = 0;
        DisplayPosition++;
    }
    if (DisplayPosition == MaxCodePosition){
        DisplayPosition = 0;
    }
    TimerDisplay++;
}

/******************************************************************************
*                                                                             *
*   Function :  int getValue_Dial_Knob_Encoder(int Minimal,                   *
*                    int Maximal, int InfinityTurn)                           *
*                                                                             *
*   Parameters : int Minimal - Minimal number to be displayed.                *
*                int Maximal - Max number to be displayed.                    *
*                int InfinityTurn - Enable infinityTurning                    *
*                                                                             *
*   Return type : dial_knob_encoder_value                                     *
*                                                                             *
*   Description : A function that will check if the new turned number is in   *
*                 range of the filled in boundaries.                          *
*                                                                             *
*******************************************************************************/
int getValue_Dial_Knob_Encoder(int Minimal, int Maximal, int InfinityTurn){
    int currentValue = getValue_Dial_Knob_Encoder();
    {
        if (currentValue < Minimal && InfinityTurn == 0) dial_knob_encoder_value = Minimal;
        else if (currentValue > Maximal && InfinityTurn == 0) dial_knob_encoder_value = Maximal;
        else if (currentValue < Minimal && InfinityTurn == 1) dial_knob_encoder_value = Maximal;
        else if (currentValue > Maximal && InfinityTurn == 1) dial_knob_encoder_value = Minimal;
    }
    return dial_knob_encoder_value;
}

/******************************************************************************
*                                                                             *
*   Function : void setPin_Dial_Knob_Encoder()                                *
*                                                                             *
*   Parameters : none                                                         *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will setup all pins for the Dial-Encoder    *
*                 so they defined as output and inputs.                       *
*                                                                             *
*******************************************************************************/
void setPin_Dial_Knob_Encoder(){
    pinMode(DIAL_ENCODER_INPUT_PINA, INPUT);
    pinMode(DIAL_ENCODER_INPUT_PINB, INPUT);
    aLastState = digitalRead(DIAL_ENCODER_INPUT_PINA);
}

/******************************************************************************
*                                                                             *
*   Function : int getValue_Dial_Knob_Encoder()                               *
*                                                                             *
*   Parameters : none                                                         *
*                                                                             *
*   Return type : dial_knob_encoder_value                                     *
*                                                                             *
*   Description : A function that will check new value from the Dial-Knop     *
*                 and count it up or down depence on the turn and return      *
*                 the value it is changed to.                                 *
*                                                                             *
*******************************************************************************/
int getValue_Dial_Knob_Encoder(){
    aState = digitalRead(DIAL_ENCODER_INPUT_PINA);
    if (aState != aLastState){
        if (digitalRead(DIAL_ENCODER_INPUT_PINB) != aState && aState != aLastState) {
            dial_knob_encoder_value = dial_knob_encoder_value + 0.5;
        } else if (aState != aLastState) {
            dial_knob_encoder_value = dial_knob_encoder_value - 0.5;
        }
    }
    aLastState = aState;
    return dial_knob_encoder_value;
}

/******************************************************************************
*                                                                             *
*   Function : void setPin_Dial_Knob_Button()                                 *
*                                                                             *
*   Parameters : none                                                         *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will setup all pins for the Dial-Button     *
*                 so they defined as input.                                   *
*                                                                             *
*******************************************************************************/
void setPin_Dial_Knob_Button(){
    pinMode(DIAL_BUTTON_INPUT_PIN, INPUT);
}

/******************************************************************************
*                                                                             *
*   Function : int getValue_Dial_Button(int WaitUntilSwitchOff)               *
*                                                                             *
*   Parameters : int WaitUntilSwitchOff - Enable parameter                    *
*                                                                             *
*   Return type : DIAL_BUTTON_VALUE                                           *
*                                                                             *
*   Description : A function that will check if the Dial-Button is pressed    *
 *                if so it will also checks if it is not hold so it will not  *
 *                reset the button is pressed signal                          *
*                                                                             *
*******************************************************************************/
int getValue_Dial_Button(int WaitUntilSwitchOff){
    int DIAL_BUTTON_VALUE = digitalRead(DIAL_BUTTON_INPUT_PIN);
    if (WaitUntilSwitchOff){
        if (LastValueDailButton != DIAL_BUTTON_VALUE){
            LastValueDailButton = DIAL_BUTTON_VALUE;
        } else if (DIAL_BUTTON_VALUE){
            DIAL_BUTTON_VALUE = 0;
        }
        return DIAL_BUTTON_VALUE;
    } else {
        return DIAL_BUTTON_VALUE;
    }
}

/******************************************************************************
*                                                                             *
*   Function : void set_DoorCloseSensor()                                     *
*                                                                             *
*   Parameters : none                                                         *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will setup all pins for the Door-Button     *
*                 so they defined as input.                                   *
*                                                                             *
*******************************************************************************/
void set_DoorCloseSensor(){
    pinMode(doorclose_switch_pin, INPUT);
}

/******************************************************************************
*                                                                             *
*   Function : int get_DoorCloseSensor()                                      *
*                                                                             *
*   Parameters : none                                                         *
*                                                                             *
*   Return type : Door_Button_Value - Open [1] / Closed [0]                   *
*                                                                             *
*   Description : A function that will read the door close button.            *
*                 And return the value.                                       *
*                                                                             *
*******************************************************************************/
int get_DoorCloseSensor(){
    int Door_Button_Value = digitalRead(doorclose_switch_pin);
    return Door_Button_Value;
}

/******************************************************************************
*                                                                             *
*   Function : void set_Servo()                                               *
*                                                                             *
*   Parameters : none                                                         *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will setup all pins for the servo           *
*                 so they defined as input. And will close the safe           *
*                                                                             *
*******************************************************************************/
void set_Servo() {
    myservo.attach(SERVO_PWM_PIN);
    move_Servo(0);
}

/******************************************************************************
*                                                                             *
*   Function : void move_Servo(int open)                                      *
*                                                                             *
*   Parameters : int open - variable if it need to be open or closed          *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will open or close the safe with the servo. *
*                                                                             *
*******************************************************************************/
void move_Servo(int open){
    if (open) myservo.write(90);
    if (!open) myservo.write(0);
}

/******************************************************************************
*                                                                             *
*   Function : void setBuzzer()                                               *
*                                                                             *
*   Parameters : none                                                         *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will setup all pins for the Buzzer          *
*                 so they defined as output.                                  *
*                                                                             *
*******************************************************************************/
void setBuzzer(){
    pinMode(BUZZER_PIN, OUTPUT);
}

/******************************************************************************
*                                                                             *
*   Function : void setBuzzerOn(int Aan)                                      *
*                                                                             *
*   Parameters : int Aan - variable if it need to be on or off                *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will active the alarm or disable the alarm. *
*                                                                             *
*******************************************************************************/
g

/******************************************************************************
*                                                                             *
*   Function : void setPirsensorPins()                                        *
*                                                                             *
*   Parameters : none                                                         *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will setup all pins for the Pir-sensor      *
*                 so they defined as output and inputs.                       *
*                                                                             *
*******************************************************************************/
void setPirsensorPins(){
    pinMode(Led1Pin1, OUTPUT);
    pinMode(Led1Pin2, OUTPUT);
    pinMode(PirSensorPin, INPUT);
    pinMode(PirEnableButton, INPUT);
    pinMode(ResetCodeButton, INPUT);
}

/******************************************************************************
*                                                                             *
*   Function : void checkPirButton()                                          *
*                                                                             *
*   Parameters : none                                                         *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will check the pir button and change        *
*                 the state of the pir sensor if needed.                      *
*                                                                             *
*******************************************************************************/
void checkPirButton() {
    if (digitalRead(PirEnableButton) == HIGH && btn_prev == LOW) //> 555
    {
        if (pirEnabled){
            pirEnabled = 0;
        }
        else if (!pirEnabled) {
            pirEnabled = 1;
        }
    }
    btn_prev = digitalRead(PirEnableButton);

    if (pirEnabled == 1){
        analogWrite(Led1Pin2, 254);
    }
    if (pirEnabled == 0) {
        analogWrite(Led1Pin2, 0);
    }
}

/******************************************************************************
*                                                                             *
*   Function : void usePirSensor()                                          *
*                                                                             *
*   Parameters : none                                                         *
*                                                                             *
*   Return type : none                                                        *
*                                                                             *
*   Description : A function that will checks if something is moving in front *
*                 of the pir sensor and if so it will start the delay.        *
*                 If the delay expires it will trigger the alarm system.      *
*                                                                             *
*******************************************************************************/
void usePirSensor() {
    if (pirEnabled) {
        if (digitalRead(PirSensorPin) && pirSensorActivityDetected == 0) {
            pirSensorActivityDetected = 1;
        }
        if (pirSensorActivityDetected && !doorState && !validateCode() && delayTimer2 == NULL) {
            delayTimer2 = millis();
        }
        if (pirSensorActivityDetected && !doorState && !validateCode() && ((millis() - delayTimer2) >= 10000)) {
            delayTimer2 = NULL;
            setBuzzerOn(1);
            delay(1000);
            pirSensorActivityDetected = 0;
        } else {
            setBuzzerOn(0);
        }
    } else {
        delayTimer2 = NULL;
    }
}
