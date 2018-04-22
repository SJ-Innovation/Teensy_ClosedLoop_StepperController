
#include <Arduino.h>
#include "AccelStepper.h"
#include "pins.h"
#include "QuadDecode_Def.h"


QuadDecode<1> X_FTM;
QuadDecode<2> Y_FTM;

AccelStepper X_Motor(AccelStepper::DRIVER, X_STEP_OUT_PIN, X_DIR_OUT_PIN);
AccelStepper Y_Motor(AccelStepper::DRIVER, Y_STEP_OUT_PIN, Y_DIR_OUT_PIN);

int_fast32_t DirStates[2];
int32_t StepperSetpoint[2] = {0, 0};

//u_int32_t StepInputCounts[2] = {0,0};
void XStepIn_ISR() {
    StepperSetpoint[X_AXIS] += DirStates[X_AXIS];
    //  StepInputCounts[X_AXIS]++;
}

void XDirIn_ISR() {
    DirStates[X_AXIS] = digitalReadFast(X_DIR_IN_PIN) ? 1 : -1;
}

void YStepIn_ISR() {
    StepperSetpoint[Y_AXIS] += DirStates[Y_AXIS];
    //StepInputCounts[Y_AXIS]++;
}

void YDirIn_ISR() {
    DirStates[Y_AXIS] = digitalReadFast(Y_DIR_IN_PIN) ? -1 : 1;
}

void pit1_isr(){
    X_Motor.run();
    Y_Motor.run();
    PIT_TFLG2 = PIT_TFLG_TIF;
}

void InitPIT(){
   // noInterrupts();
    SIM_SCGC6 |= SIM_SCGC6_PIT;
    PIT_MCR = 0;
    PIT_LDVAL0 = F_BUS / (MOTOR_STEPS_PER_MM*MAX_SPEED_MMS);
    PIT_TCTRL0 = PIT_TCTRL_TEN| PIT_TCTRL_TIE;
  //  interrupts();
}

void Loop() {
    static bool LastEnableState = LOW;
    bool EnableState = digitalReadFast(ENABLE_PIN);
    digitalWriteFast(LED_BUILTIN, EnableState);
    if (EnableState) {
        if (EnableState != LastEnableState) {
            X_Motor.setCurrentPosition(0);
            X_FTM.zeroFTM();
            StepperSetpoint[X_AXIS] = 0;
            Y_Motor.setCurrentPosition(0);
            Y_FTM.zeroFTM();
            StepperSetpoint[Y_AXIS] = 0;
        }
        int32_t EncoderX = X_FTM.calcPosn();
        int32_t RealX = round(ENCODER_TO_REAL(EncoderX));
        if (abs(RealX - X_Motor.currentPosition()) >= 3) { // TODO SPEED DEPENDANT
            X_Motor.setCurrentPosition(RealX);
        }
        int32_t EncoderY = Y_FTM.calcPosn();
        int32_t RealY = round(ENCODER_TO_REAL(EncoderY));
        if (abs(RealY - Y_Motor.currentPosition()) >= 3) {
            Y_Motor.setCurrentPosition(RealY);
        }
#if 0 // Debug
        static u_int32_t C = 0;
        static u_int32_t Time = millis();
        C++;
        if (millis() - Time >= 200) {//
            Serial.println(C * 5);
            C = 0;
            Serial.print("X ENC:");
            Serial.println(EncoderX);
            Serial.print("X REAL:");
            Serial.println(RealX);
            Serial.print("X CURR:");
            Serial.println(X_Motor.currentPosition());
            Serial.print("X TGT:");
            Serial.println(X_Motor.targetPosition());
            Serial.print("X SPD:");
            Serial.println(X_Motor.speed());
            Serial.print("Y ENC:");
            Serial.println(EncoderY);
            Serial.print("Y REAL:");
            Serial.println(RealY);
            Serial.print("Y CURR:");
            Serial.println(Y_Motor.currentPosition());
            Serial.print("Y TGT:");
            Serial.println(Y_Motor.targetPosition());
            Serial.println(" ");
            digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
            Time = millis();
        }
#endif //Debug
    }
    LastEnableState = EnableState;
    X_Motor.moveTo(StepperSetpoint[X_AXIS]);
  //  X_Motor.run();

    Y_Motor.moveTo(StepperSetpoint[Y_AXIS]);
   // Y_Motor.run();


}


void Setup() {
    pinMode(LED_BUILTIN, OUTPUT);
  //  Serial.begin(115200);
    pinMode(ENABLE_PIN, INPUT);
    pinMode(X_STEP_IN_PIN, INPUT);
    pinMode(X_DIR_IN_PIN, INPUT);
    pinMode(Y_STEP_IN_PIN, INPUT);
    pinMode(Y_DIR_IN_PIN, INPUT);
    pinMode(X_STEP_OUT_PIN, OUTPUT_STRONGDRIVE);
    pinMode(X_DIR_OUT_PIN, OUTPUT_STRONGDRIVE);
    pinMode(Y_STEP_OUT_PIN, OUTPUT_STRONGDRIVE);
    pinMode(Y_DIR_OUT_PIN, OUTPUT_STRONGDRIVE);
    digitalWriteFast(X_STEP_OUT_PIN, LOW);
    digitalWriteFast(Y_STEP_OUT_PIN, LOW);
    digitalWriteFast(X_DIR_OUT_PIN, LOW);
    digitalWriteFast(Y_DIR_OUT_PIN, LOW);
    X_FTM.setup();
    Y_FTM.setup();
    X_FTM.start();
    Y_FTM.start();
    X_FTM.zeroFTM();
    Y_FTM.zeroFTM();
    X_Motor.setCurrentPosition(0);
    Y_Motor.setCurrentPosition(0);
    X_Motor.setMaxSpeed(MAX_SPEED_MMS * MOTOR_STEPS_PER_MM);
    Y_Motor.setMaxSpeed(MAX_SPEED_MMS * MOTOR_STEPS_PER_MM);
    X_Motor.setAcceleration(MAX_ACCEL_MMSS * MOTOR_STEPS_PER_MM);
    Y_Motor.setAcceleration(MAX_ACCEL_MMSS * MOTOR_STEPS_PER_MM);
    X_Motor.setPinsInverted(false, false, false);
    Y_Motor.setPinsInverted(true, false, false);
    attachInterrupt(X_STEP_IN_PIN, XStepIn_ISR, RISING);
    attachInterrupt(Y_STEP_IN_PIN, YStepIn_ISR, RISING);
    attachInterrupt(X_DIR_IN_PIN, XDirIn_ISR, CHANGE);
    attachInterrupt(Y_DIR_IN_PIN, YDirIn_ISR, CHANGE);
    XDirIn_ISR(); //Prime Direction ISRs
    YDirIn_ISR();
    InitPIT();
    interrupts();
    pinMode(LED_BUILTIN, OUTPUT_STRONGDRIVE);
    digitalWriteFast(LED_BUILTIN, HIGH);
}

int main() {
    Setup();
    while (1) {
        Loop();
    }

}

//