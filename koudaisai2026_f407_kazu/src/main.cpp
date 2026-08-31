#include <Arduino.h>
#include "ps3_sbdbt.h"
#include "pin_define.h"

using namespace raven;

#define FAMIMA 0 // 起動音のONOFF
#define UNICORN     0

#define DAIHIHOU_POSITION 510 // 大秘宝を獲得するアームのAD値,ボタン一つでここまで移動させる: 値を上げるとアームが上がる、下げるとアームも下がる
#define COALA_POSITION 460
#define ARM_UNDER_LIMIT 460 // アームの最下点
#define ARM_UPPER_LIMIT 700 // アームの最上点

const int OFFSET = 10;

/* PS3 joystick limits defination: */
#define X_MAX 100
#define X_MIN 20
#define Y_MAX 100
#define Y_MIN 20

#define CTRL_GAIN 2.0f // 操作ゲイン(joystick)

#define COS30 0.57

int myled0 = PE_0;
int myled1 = PE_1;
int myled2 = PE_2;
int myled3 = PE_3;

int ledUp      = PE_8;
int ledDown    = PE_10;
int ledRight   = PE_7;
int ledLeft    = PE_12;
int RESET_LED  = PE_4;

int out_A3 = PE_13;
int out_B3 = PE_14;

int arm_A = PA_5;
int arm_B = PA_3;

int hand_A = PD_7;
int hand_B = PD_4;

int BACKLIGHT = PA_6;

int out_A1 = PC_7;
int out_A2 = PC_8;
int out_B1 = PC_6;
int out_B2 = PC_9;

int BZ = PB_15;

int RED = PB_9;
int GREEN = PB_8;
int BLUE = PB_14;

int VR_pin = PC_0;

// SBDBT = Serial1 (PA9 = TX, PA10 = RX)
SBDBT sbdbt(PA10_ALT0, PA9_ALT0, 2400);

// DebugSerial = PC11 (RX), PC10 (TX)
HardwareSerial DebugSerial(PC11_ALT0, PC10_ALT0);

// 0.5ms割り込み
HardwareTimer *timer1;
HardwareTimer *pwmTimer = new HardwareTimer(TIM3);
HardwareTimer *pwm1 = new HardwareTimer(TIM1);
HardwareTimer *pwm2 = new HardwareTimer(TIM2);

int bpm=64;
int daihihou_flag = 0;  // 1にするとアーム位置を大秘宝位置に強制移動
int coala_flag = 0;  // 1にするとアーム位置を秘宝位置に強制移動
int jimen_flag = 0;  // 1にするとアーム位置を地面に強制移動
int now_vri = 0;  // VRの現在位置(AD)

unsigned char isBoost = 0; // ブーストが押されてるかどうか
unsigned char isCircle = 0;
unsigned char isOpen = 0, isClose = 0;  // アームの開閉状態
unsigned char isDaihihou = 0;  // 大秘宝の位置にアームを移動するか
unsigned char isCoala = 0;
unsigned char isUnder = 0;  // 地面すれすれに移動

float rx = 0.0f, ry = 0.0f, lx = 0.0f, ly = 0.0f;
float cosA = 0.0f,cosB = 0.0f,cosC = 0.0f;  // 各ホイールとの角度の比
float f1 = 0.0f, f2 = 0.0f, f3 = 0.0f;  // モータにかける最終pwm
float theta = 0.0f;  // 中心からのずれ
float ctrl_abs = 0.0f; // 方角(絶対値)
float cnt0 = 0.0f, cnt1 = 0.0f;
float duty0 = 0.0f;
float omega = 0.0f;
float omega_filtered = 0.0f; // 慣性フィルタ用
float straight_gain = 0.0f; // 直進補正用

// ハンド:疑似PWM
const int INTERVAL = 100;
int j = 0;
int hand_on = 0;
unsigned long cnt_h= 0;

void motor1(float pwm );
void motor2(float pwm );
void motor3(float pwm );
void arm(float pwm );
void interrupt_01ms( void );
void setColor( int state_r, int state_g, int state_b );
void DigitalArm( float duty0 );
void armPID( void );
float mapf(float x, float in_min, float in_max, float out_min, float out_max);
float expo(float x, float e);
void setup()
{
    // DigitalOut
    pinMode(myled0, OUTPUT);
    pinMode(myled1, OUTPUT);
    pinMode(myled2, OUTPUT);
    pinMode(myled3, OUTPUT);

    pinMode(ledUp, OUTPUT);
    pinMode(ledDown, OUTPUT);
    pinMode(ledRight, OUTPUT);
    pinMode(ledLeft, OUTPUT);
    pinMode(RESET_LED, OUTPUT);
    pinMode(arm_A, OUTPUT);
    pinMode(arm_B, OUTPUT);
    pinMode(hand_A, OUTPUT);
    pinMode(hand_B, OUTPUT);

    // PWM
    pinMode(BACKLIGHT, OUTPUT);
    pinMode(out_A1, OUTPUT);
    pinMode(out_A2, OUTPUT);
    pinMode(out_B1, OUTPUT);
    pinMode(out_B2, OUTPUT);
    pinMode(out_A3, OUTPUT);
    pinMode(out_B3, OUTPUT);

    pinMode(BZ, OUTPUT);
    pinMode(RED, OUTPUT);
    pinMode(GREEN, OUTPUT);
    pinMode(BLUE, OUTPUT);

    // AnalogIn
    pinMode(VR_pin, INPUT_ANALOG);
    DebugSerial.begin(9600);
    DebugSerial.println("SBDBT driver started.");

    timer1 = new HardwareTimer(TIM1); // TIM1をタイマー割込みとして使う
    timer1->setOverflow(500, MICROSEC_FORMAT); // 0.5ms = 500µs 周期
    timer1->attachInterrupt(interrupt_01ms); // 割り込み関数を登録
    timer1->resume();  // タイマー開始

    // 足回りモータのpwm周期を125kHzに設定
    pwmTimer->setOverflow(125000, HERTZ_FORMAT);  // 125kHz
    pwmTimer->resume();
    pwm1->setOverflow(125000, HERTZ_FORMAT);  // 125kHz
    pwm1->resume();
    // アーム上下モータ用
    pwm2->setOverflow(125000, HERTZ_FORMAT);  // 125kHz
    pwm2->resume();

    digitalWrite(RESET_LED, HIGH); // 準備完了
}


void loop()
{
    // SBDBTからの入力受け取り
    sbdbt.update();
    if (!sbdbt.available()) return;
    
    // VR（アナログ入力）
    now_vri = (analogRead(VR_pin));   // 12bit → 10bit
    
    float now_vr = mapf(now_vri, 0, 1023, -1.0f, 1.0f);
    DebugSerial.print("now_vri= ");
    DebugSerial.println(now_vri);

    // sbdbt入力（-1.0 ～ +1.0）
    rx = mapf(sbdbt.rs_x(), 0.0f,128.0f, -1.0f,1.0f);
    ry = mapf(sbdbt.rs_y(), 0.0f,128.0f, -1.0f,1.0f);
    lx = mapf(sbdbt.ls_x(), 0.0f,128.0f, -1.0f,1.0f);
    ly = mapf(sbdbt.ls_y(), 0.0f,128.0f, -1.0f,1.0f);

    // 移動量の指数関数補正
    lx = expo(lx, 0.45f);   // 左右
    ly = expo(ly, 0.35f);   // 前後
    rx = expo(rx, 0.25f);   // 回転

    isBoost  = sbdbt.L2();
    isCircle = sbdbt.L1();
    isOpen   = sbdbt.batu();
    isClose  = (sbdbt.maru() | sbdbt.R2());
    isDaihihou = sbdbt.sikaku();
    isCoala    = sbdbt.sankaku();
    isUnder    = sbdbt.sita();

    // LED色設定
    if(isBoost)          setColor(1,1,1);
    else if(isCoala)     setColor(0,1,0);
    else if(isDaihihou)  setColor(1,0,1);
    else if(isOpen)      setColor(0,0,1);
    else if(isClose)     setColor(1,0,0);
    else if(isUnder)     setColor(1,1,0);
    else                 setColor(0,0,0);

    // デッドゾーン処理
    if(sbdbt.rs_x() == 64) rx = 0.0f;
    if(sbdbt.rs_y() == 64) ry = 0.0f;
    if(sbdbt.ls_x() == 64) lx = 0.0f;
    if(sbdbt.ls_y() == 64) ly = 0.0f;

    // 移動量の指数関数補正
    lx = expo(lx, 0.45f);
    ly = expo(ly, 0.35f);
    rx = expo(rx, 0.25f);

    // 直進補正（前進時（lyの絶対値が大きい時）にlxを自動で弱める）
    straight_gain = 1.0f - fabs(ly);
    lx *= straight_gain;

    // 通常動作の設定
    theta = atan2(lx, ly);
    ctrl_abs = CTRL_GAIN * sqrt(lx*lx + ly*ly);

    cosA = ctrl_abs * cos( 30 * M_PI / 180 - theta);
    cosB = ctrl_abs * cos(150 * M_PI / 180 - theta);
    cosC = ctrl_abs * cos(270 * M_PI / 180 - theta);
    //cosC = (ctrl_abs/CTRL_GAIN)*lx;

    f1 = mapf(cosA, -1.42f, 1.42f,  1.0f, -1.0f);
    f2 = mapf(cosB, -1.42f, 1.42f, -1.0f,  1.0f);
    f3 = mapf(cosC, -1.42f, 1.42f, -1.0f,  1.0f);

    // 右スティックで回転操作と組み合わせる
    omega = rx;   // rs_xから得た値
    omega_filtered = 0.9f * omega_filtered + 0.1f * omega; // 慣性フィルタ
    f1 += -1.0f*omega_filtered;
    f2 += omega_filtered;
    f3 += omega_filtered;

    // Boost処理
    if(isBoost) {
        if(sbdbt.ls_x() > X_MAX) { // 右移動
            f1 = -1.0f * cos(60*(M_PI/180)); 
            f2 = 1.0f * cos(60*(M_PI/180)); 
            f3 = -1.0f;
        }
        else if(sbdbt.ls_x() < X_MIN) { // 左移動 
            f1 = 1.0f * cos(60*(M_PI/180)); 
            f2 = -1.0f * cos(60*(M_PI/180));
            f3 = 1.0f;
        }
        else if(sbdbt.rs_x() > X_MAX) { // 右回転
            f1 = -1.0f;
            f2 = 1.0f;
            f3 = 1.0f;
        }
        else if(sbdbt.rs_x() < X_MIN) { // 左回転
            f1 = 1.0f;
            f2 = -1.0f;
            f3 = -1.0f;
        }
        else if(sbdbt.ls_y() > Y_MAX) { // 前進
            f1 = -1.0f;
            f2 = -1.0f; 
            f3 = 0.0f;
        }
        else if(sbdbt.ls_y() < Y_MIN) { // 後退
            f1 = 1.0f; 
            f2 = 1.0f; 
            f3 = 0.0f;
        }
        omega *= 2.0f;
    }

    // フラグ処理
    daihihou_flag = isDaihihou ? 1 : 0;
    coala_flag    = isCoala    ? 1 : 0;
    jimen_flag    = isUnder    ? 1 : 0;

    // アームの位置を維持したまま足回りだけ動かす
    if(isCircle) {

        // アーム先端固定のための補正回転
        float omega_fix = lx / 0.55f;   // 550mm = 0.55m

        // 横移動ベクトル（通常の横移動）
        float vx = lx;
        float vy = 0.0f;

        float thA = 30.0f * M_PI / 180.0f;
        float thB = 150.0f * M_PI / 180.0f;
        float thC = 270.0f * M_PI / 180.0f;

        f1 = lx * cos(thA) + vy * sin(thA);
        f2 = lx * cos(thB) + vy * sin(thB);
        f3 = lx * cos(thC) + vy * sin(thC);

        // アーム先端固定のための逆回転成分
        f1 += -omega_fix;
        f2 +=  omega_fix;
        f3 +=  omega_fix;

        DebugSerial.print("f1= ");
        DebugSerial.print(f1);
        DebugSerial.print(" f2= ");
        DebugSerial.print(f2);
        DebugSerial.print(" f3= ");
        DebugSerial.println(f3);
    }


    // ハンド速度調整
    j = INTERVAL - 3;

    if(isOpen && !isClose) {
        hand_on = 1;
    } else if(!isOpen && isClose) {
        hand_on = 2;
    } else {
        hand_on = 0;
        digitalWrite(hand_A, LOW);
        digitalWrite(hand_B, LOW);
    }

    // モーター動作
    motor1(f1);
    motor2(f2);
    motor3(-1.0f * f3);

    // アーム動作
    armPID();
    DigitalArm(ry);
}


void interrupt_01ms(void)
{
    cnt_h++;
    cnt0 += 0.05f;
    if(cnt0 > 1.0f) cnt0 = 0.0f;

    cnt1 += 0.1f;
    if(cnt1 > 1.0f) cnt1 = 0.0f;

    if(cnt_h >= INTERVAL) cnt_h = 0;

    if(cnt_h > (INTERVAL - j)) {
        if(hand_on == 1) {
            digitalWrite(hand_A, LOW);
            digitalWrite(hand_B, HIGH);
        } else if(hand_on == 2) {
            digitalWrite(hand_A, HIGH);
            digitalWrite(hand_B, LOW);
        } else {
            digitalWrite(hand_A, LOW);
            digitalWrite(hand_B, LOW);
        }
    }

    if(cnt_h > j) {
        digitalWrite(hand_A, LOW);
        digitalWrite(hand_B, LOW);
    }
}

// 範囲: -1で逆転: 0で停止: 1で正転
void motor1(float pwm)
{
    float duty;

    if (pwm == 0.0f) {
        duty = 0.5f;   // 停止
    } else {
        duty = mapf(pwm, -1.0f, 1.0f, 0.0f, 1.0f);
    }

    if (duty >= 1.0f) duty = 0.95f;
    if (duty <= 0.0f) duty = 0.05f;

    analogWrite(out_A1, duty * 255);        // 正転側
    analogWrite(out_B1, (1.0f - duty) * 255); // 逆転側
}

void motor2(float pwm)
{
    float duty;

    if (pwm == 0.0f) {
        duty = 0.5f;
    } else {
        duty = mapf(pwm, -1.0f, 1.0f, 0.0f, 1.0f);
    }

    if (duty >= 1.0f) duty = 0.95f;
    if (duty <= 0.0f) duty = 0.05f;

    analogWrite(out_A2, duty * 255);
    analogWrite(out_B2, (1.0f - duty) * 255);
}

void motor3(float pwm)
{
    float duty;

    if (pwm == 0.0f) {
        duty = 0.5f;
    } else {
        duty = mapf(pwm, -1.0f, 1.0f, 0.0f, 1.0f);
    }

    if (duty >= 1.0f) duty = 0.95f;
    if (duty <= 0.0f) duty = 0.01f;

    analogWrite(out_A3, duty * 255);         // PE13 → TIM1_CH3
    analogWrite(out_B3, (1.0f - duty) * 255); // PE14 → TIM1_CH4
}


void armPID(void)
{
    // 大秘宝位置へ強制移動
    if(daihihou_flag == 1) {
        if(now_vri < (DAIHIHOU_POSITION + OFFSET)) {
            ry = 1.0f;
        } else if(now_vri > (DAIHIHOU_POSITION - OFFSET)) {
            ry = -1.0f;
        } else {
            ry = 0.0f;
            daihihou_flag = 0;
        }
    }

    // コアラ位置へ強制移動
    if(coala_flag == 1) {
        if(now_vri < (COALA_POSITION + OFFSET)) {
            ry = 1.0f;
        } else if(now_vri > (COALA_POSITION - OFFSET)) {
            ry = -1.0f;
        } else {
            ry = 0.0f;
            coala_flag = 0;
        }
    }

    // 地面位置へ強制移動
    if(jimen_flag == 1) {
        if(now_vri < (ARM_UNDER_LIMIT + OFFSET)) {
            ry = 1.0f;
        } else if(now_vri > (ARM_UNDER_LIMIT - OFFSET)) {
            ry = -1.0f;
        } else {
            ry = 0.0f;
            jimen_flag = 0;
        }
    }
}
void DigitalArm(float duty0)
{
    if(duty0 <= -1.0f) duty0 = -0.99f;
    if(duty0 >=  1.0f) duty0 =  0.99f;

    if(now_vri <= ARM_UNDER_LIMIT && !isCoala && !isDaihihou) {
        DebugSerial.println("!!!UNDER LIMIT!!!");
        if(duty0 > 0){
            analogWrite(arm_A, 0);
            analogWrite(arm_B, 0);
            return;
        }
    }
    if(now_vri >= ARM_UPPER_LIMIT && !isCoala && !isDaihihou) {
        DebugSerial.println("!!!UNDER LIMIT!!!");
        if(duty0 < 0){
            analogWrite(arm_A, 0);
            analogWrite(arm_B, 0);
            return;
        }
    }

    if(duty0 > 0.0f) {
        float duty = duty0;
        analogWrite(arm_A, 0);
        analogWrite(arm_B, duty * 255);
    }
    else if(duty0 < 0.0f) {
        float duty = fabs(duty0);
        analogWrite(arm_A, duty * 255);
        analogWrite(arm_B, 0);
    }
    else {
        analogWrite(arm_A, 0);
        analogWrite(arm_B, 0);
    }
}

void setColor( int state_r, int state_g, int state_b )
{
    digitalWrite(RED, state_r);
    digitalWrite(GREEN, state_g);
    digitalWrite(BLUE, state_b);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// 移動量の指数関数補正
float expo(float x, float e) {
    return x * (e * x * x + (1.0f - e));
}