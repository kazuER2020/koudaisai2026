#include <Arduino.h>
#include "ps3_sbdbt.h"
#include "pin_define.h"

using namespace raven;

#define FAMIMA 0 // 起動音のONOFF
#define UNICORN     0

#define DAIHIHOU_POSITION 510 // 大秘宝を獲得するアームのAD値,ボタン一つでここまで移動させる: 値を上げるとアームが上がる、下げるとアームも下がる
#define COALA_POSITION 460
#define ARM_UNDER_LIMIT 400 // アームの最下点
#define ARM_UPPER_LIMIT 658 // アームの最上点

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

int arm_A = PA_5;   // TIM2_CH1
int arm_B = PA_3;   // TIM2_CH4

int hand_A = PD_7;  // digitalのみ
int hand_B = PD_4;  // digitalのみ

int BACKLIGHT = PA_6;

int out_A1 = PC_7;  // TIM8_CH2
int out_A2 = PC_8;  // TIM8_CH3
int out_B1 = PC_6;  // TIM8_CH1
int out_B2 = PC_9;  // TIM8_CH4
int out_A3 = PE_13; // TIM1_CH3
int out_B3 = PE_14; // TIM1_CH4

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
HardwareTimer *pwmTimer = new HardwareTimer(TIM8);
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
unsigned char isStop = 0;

float rx = 0.0f, ry = 0.0f, lx = 0.0f, ly = 0.0f;
float cosA = 0.0f,cosB = 0.0f,cosC = 0.0f;  // 各ホイールとの角度の比
float f1 = 0.0f, f2 = 0.0f, f3 = 0.0f;  // モータにかける最終pwm
float theta = 0.0f;  // 中心からのずれ
float ctrl_abs = 0.0f; // 方角(絶対値)
float cnt0 = 0.0f, cnt1 = 0.0f;
float duty0 = 0.0f;
float omega = 0.0f;
float omega_filtered = 0.0f; // 慣性フィルタ用
float straight_gain_x = 0.0f, straight_gain_y = 0.0f; // 直進補正用
float lx_filtered = 0.0f;
float ly_filtered = 0.0f;
static float lpf_state = 0.0f; // AD入力平均化用LPF


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
void PwmArm(float pwm);
unsigned int readAnalogAveraged10bit(int pin);
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
    analogReadResolution(10); // analogReadの戻り値を10bitとする

    timer1 = new HardwareTimer(TIM4); // TIM4をタイマー割込みとして使う
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
    // now_vri = analogRead(VR_pin);   // 10bit
    now_vri = readAnalogAveraged10bit(VR_pin);

    float now_vr = mapf(now_vri, 0, 1023, -1.0f, 1.0f);
    DebugSerial.print("now_vri= ");
    DebugSerial.print(now_vri);
    DebugSerial.print(", ");
    DebugSerial.print("now_vr= ");
    DebugSerial.println(now_vr);

    // sbdbt入力（-1.0 ～ +1.0）
    rx = mapf(sbdbt.rs_x(), 0.0f,128.0f, -1.0f,1.0f);
    ry = mapf(sbdbt.rs_y(), 0.0f,128.0f, -1.0f,1.0f);
    lx = mapf(sbdbt.ls_x(), 0.0f,128.0f, -1.0f,1.0f);
    ly = mapf(sbdbt.ls_y(), 0.0f,128.0f, -1.0f,1.0f);

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
    
    // 通常動作の設定
    // 直進前後補正（前進後退時（lyの絶対値が大きい時）にlxを自動で弱める）
    straight_gain_x = 1.0f - fabs(ly);
    lx *= straight_gain_x;
    lx_filtered = 0.85f * lx_filtered + 0.15f * lx; //  横移動を滑らかにするフィルタ

    // 直進左右補正（左右移動時（lxの絶対値が大きい時）にlyを自動で弱める）
    straight_gain_y = 1.0f - fabs(lx);
    ly *= straight_gain_y;
    ly_filtered = 0.85f * ly_filtered + 0.15f * ly; //  縦移動を滑らかにするフィルタ

    // theta = atan2(lx, ly);
    //ctrl_abs = CTRL_GAIN * sqrt(lx*lx + ly*ly);
    theta = atan2(lx_filtered, ly_filtered); // 補正後の値を使う
    ctrl_abs = CTRL_GAIN * sqrt(lx_filtered*lx_filtered + ly_filtered*ly_filtered); // 補正後の値を使う

    cosA = ctrl_abs * cos( 30 * M_PI / 180 - theta);
    cosB = ctrl_abs * cos(150 * M_PI / 180 - theta);
    cosC = ctrl_abs * cos(270 * M_PI / 180 - theta);
    //cosC = (ctrl_abs/CTRL_GAIN)*lx;

    f1 = mapf(cosA, -1.42f, 1.42f,  1.0f, -1.0f);
    f2 = mapf(cosB, -1.42f, 1.42f, -1.0f,  1.0f);
    f3 = mapf(cosC, -1.42f, 1.42f, -1.0f,  1.0f);
 
    // 右スティックで回転操作と組み合わせる
    omega = rx;   // rs_xから得た値
    omega_filtered = 0.85f * omega_filtered + 0.15f * omega; // 慣性フィルタ
    f1 += -1.0f*omega_filtered;
    f2 += omega_filtered;
    f3 += omega_filtered;

    // Boost処理
    /*
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
    */

    // フラグ処理
    daihihou_flag = isDaihihou ? 1 : 0;
    coala_flag    = isCoala    ? 1 : 0;
    jimen_flag    = isUnder    ? 1 : 0;

    if(isCircle){
        float corr = lx_filtered * 0.1f;
        // 左右判定を角度で行う
        float angle = atan2(ly_filtered, lx_filtered);  // -180°〜+180°

        // 右方向（角度が -45°〜+45°）
        if(angle > -0.785f && angle < 0.785f){
            f3 = -fabs(lx_filtered);   // motor3(-f3) → +main
            f1 = -fabs(corr);
            f2 =  fabs(corr);
        }
        // 左方向（角度が 135°〜180° または -180°〜-135°）
        else if(angle > 2.356f || angle < -2.356f){
            f3 =  fabs(lx_filtered);   // motor3(-f3) → -main
            f1 =  fabs(corr);
            f2 = -fabs(corr);
        }
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
    //armPID();
    // DigitalArm(ry);
    PwmArm(ry);
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

// アームをPWMで動かす。入力はジョイスティックrx座標
void PwmArm(float pwm)
{
    const float DEADZONE   = 0.05f;
    const float INPUT_LPF  = 0.20f;
    const float PWM_LPF    = 0.25f;
    const float SLEW_RATE  = 0.05f;

    static float ry_filtered = 0.0f;
    static float pwm_smooth  = 0.0f;
    static float pwm_final   = 0.0f;

    // ============================================================
    // 自動位置移動　P制御
    // ============================================================
    if(daihihou_flag == 1) {
        if(now_vri < (DAIHIHOU_POSITION - OFFSET)) pwm = 1.0f;
        else if(now_vri > (DAIHIHOU_POSITION + OFFSET)) pwm = -1.0f;
        else { pwm = 0.0f; daihihou_flag = 0; }
    }

    if(coala_flag == 1) {
        if(now_vri < (COALA_POSITION - OFFSET)) pwm = 1.0f;
        else if(now_vri > (COALA_POSITION + OFFSET)) pwm = -1.0f;
        else { pwm = 0.0f; coala_flag = 0; }
    }

    if(jimen_flag == 1) {
        if(now_vri < (ARM_UNDER_LIMIT - OFFSET)) pwm = 1.0f;
        else if(now_vri > (ARM_UNDER_LIMIT + OFFSET)) pwm = -1.0f;
        else { pwm = 0.0f; jimen_flag = 0; }
    }

    // ============================================================
    // ジョイスティック入力のLPF
    // ============================================================
    float prev_filtered = ry_filtered;
    ry_filtered = (1.0f - INPUT_LPF) * ry_filtered + INPUT_LPF * pwm;

    // ============================================================
    // 方向反転時はフィルタ即リセット
    // ============================================================
    if((prev_filtered > 0 && ry_filtered < 0) ||
       (prev_filtered < 0 && ry_filtered > 0))
    {
        pwm_smooth = ry_filtered;
        pwm_final  = ry_filtered;
    }

    // ============================================================
    // ジョイスティックが離されている場合は完全停止
    // ============================================================
    if(fabs(ry_filtered) < DEADZONE) {
        ry_filtered = 0.0f;
        pwm_smooth  = 0.0f;
        pwm_final   = 0.0f;
        analogWrite(arm_A, 0);
        analogWrite(arm_B, 0);
        return;
    }

    // ============================================================
    // 機構限界保護
    // ============================================================
    bool isAuto = (daihihou_flag || coala_flag || jimen_flag);

    if(now_vri <= ARM_UNDER_LIMIT && !isAuto) {
        if(ry_filtered < 0) {
            ry_filtered = 0.0f;
            pwm_smooth  = 0.0f;
            pwm_final   = 0.0f;
        }
    }

    if(now_vri >= ARM_UPPER_LIMIT && !isAuto) {
        if(ry_filtered > 0) {
            ry_filtered = 0.0f;
            pwm_smooth  = 0.0f;
            pwm_final   = 0.0f;
        }
    }

    // ============================================================
    // ⑥ 加速度制限
    // ============================================================
    float diff = ry_filtered - pwm_smooth;
    if(diff > SLEW_RATE) diff = SLEW_RATE;
    if(diff < -SLEW_RATE) diff = -SLEW_RATE;
    pwm_smooth += diff;

    // ============================================================
    // ⑦ PWMフィルタ
    // ============================================================
    pwm_final = (1.0f - PWM_LPF) * pwm_final + PWM_LPF * pwm_smooth;

    // ============================================================
    // ⑧ ★初動逆方向防止：pwm_final が ±0.10 未満なら完全停止
    // ============================================================
    if(fabs(pwm_final) < 0.10f) {
        analogWrite(arm_A, 0);
        analogWrite(arm_B, 0);
        return;
    }

    // ============================================================
    // ⑨ PWM駆動（mbed互換）
    // ============================================================
    float duty = mapf(pwm_final, -1.0f, 1.0f, 0.0f, 1.0f);

    if(duty >= 1.0f) duty = 0.95f;
    if(duty <= 0.0f) duty = 0.01f;

    analogWrite(arm_B, duty * 255);
    analogWrite(arm_A, (1.0f - duty) * 255);
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

inline unsigned int median3(unsigned int a, unsigned int b, unsigned int c)
{
    if(a > b) { unsigned int t = a; a = b; b = t; }
    if(b > c) { unsigned int t = b; b = c; c = t; }
    if(a > b) { unsigned int t = a; a = b; b = t; }
    return b;   // 中央値
}

// アナログ入力を複数回サンプリングして平均化する
unsigned int readAnalogAveraged10bit(int pin)
{
    const int SAMPLE_COUNT = 32;
    unsigned int sum = 0;

    // ① 32回サンプリング（10bit）
    for(int i = 0; i < SAMPLE_COUNT; i++) {

        // 3点中央値フィルタで突発ノイズを除去
        unsigned int a = analogRead(pin);
        unsigned int b = analogRead(pin);
        unsigned int c = analogRead(pin);
        unsigned int med = median3(a, b, c);
        sum += med;
    }

    // 平均化
    unsigned int avg = sum / SAMPLE_COUNT;

    // LPF
    lpf_state = 0.90f * lpf_state + 0.10f * avg;

    // 0〜1023にまるめる
    if(lpf_state < 0.0f)   lpf_state = 0.0f;
    if(lpf_state > 1023.0f) lpf_state = 1023.0f;

    return (unsigned int)(lpf_state + 0.5f);
}