#pragma once

// ===============================
// mbed互換ピン名 → Arduino STM32 (ALT0付き)
// ===============================

// ---- Port A ----
#define PA_0   PA0_ALT0
#define PA_1   PA1_ALT0
#define PA_2   PA2_ALT0
#define PA_3   PA3_ALT0
#define PA_4   PA4_ALT0
#define PA_5   PA5_ALT0
#define PA_6   PA6_ALT0
#define PA_7   PA7_ALT0
#define PA_8   PA8_ALT0
#define PA_9   PA9_ALT0
#define PA_10  PA10_ALT0
#define PA_11  PA11_ALT0
#define PA_12  PA12_ALT0
#define PA_13  PA13_ALT0
#define PA_14  PA14_ALT0
#define PA_15  PA15_ALT0

// ---- Port B ----
#define PB_0   PB0_ALT0
#define PB_1   PB1_ALT0
#define PB_2   PB2_ALT0
#define PB_3   PB3_ALT0
#define PB_4   PB4_ALT0
#define PB_5   PB5_ALT0
#define PB_6   PB6_ALT0
#define PB_7   PB7_ALT0
#define PB_8   PB8_ALT0
#define PB_9   PB9_ALT0
#define PB_10  PB10_ALT0
#define PB_11  PB11_ALT0
#define PB_12  PB12_ALT0
#define PB_13  PB13_ALT0
#define PB_14  PB14_ALT0
#define PB_15  PB15_ALT0

// ---- Port C ----
#define PC_0   PC0_ALT0
#define PC_1   PC1_ALT0
#define PC_2   PC2_ALT0
#define PC_3   PC3_ALT0
#define PC_4   PC4_ALT0
#define PC_5   PC5_ALT0
#define PC_6   PC6_ALT0
#define PC_7   PC7_ALT0
#define PC_8   PC8_ALT0
#define PC_9   PC9_ALT0
#define PC_10  PC10_ALT0
#define PC_11  PC11_ALT0
#define PC_12  PC12_ALT0
#define PC_13  PC13_ALT0
#define PC_14  PC14_ALT0
#define PC_15  PC15_ALT0

// ---- Port D ----
#define PD_0   PD0_ALT0
#define PD_1   PD1_ALT0
#define PD_2   PD2_ALT0
#define PD_3   PD3_ALT0
#define PD_4   PD4_ALT0
#define PD_5   PD5_ALT0
#define PD_6   PD6_ALT0
#define PD_7   PD7_ALT0
#define PD_8   PD8_ALT0
#define PD_9   PD9_ALT0
#define PD_10  PD10_ALT0
#define PD_11  PD11_ALT0
#define PD_12  PD12_ALT0
#define PD_13  PD13_ALT0
#define PD_14  PD14_ALT0
#define PD_15  PD15_ALT0

// ---- Port E ----
#define PE_0   PE0_ALT0
#define PE_1   PE1_ALT0
#define PE_2   PE2_ALT0
#define PE_3   PE3_ALT0
#define PE_4   PE4_ALT0
#define PE_5   PE5_ALT0
#define PE_6   PE6_ALT0
#define PE_7   PE7_ALT0
#define PE_8   PE8_ALT0
#define PE_9   PE9_ALT0
#define PE_10  PE10_ALT0
#define PE_11  PE11_ALT0
#define PE_12  PE12_ALT0
#define PE_13  PE13_ALT0
#define PE_14  PE14_ALT0
#define PE_15  PE15_ALT0

// ===============================
// DigitalOut / PwmOut / AnalogIn の mbed変数名を「変数として宣言」
// ===============================

// DigitalOut
extern int myled0;
extern int myled1;
extern int myled2;
extern int myled3;

extern int ledUp;
extern int ledDown;
extern int ledRight;
extern int ledLeft;
extern int RESET_LED;

extern int out_A3;
extern int out_B3;

extern int arm_A;
extern int arm_B;

extern int hand_A;
extern int hand_B;

// PWM
extern int BACKLIGHT;

extern int out_A1;
extern int out_A2;
extern int out_B1;
extern int out_B2;

extern int BZ;

extern int RED;
extern int GREEN;
extern int BLUE;

// AnalogIn
extern int VR_pin;
