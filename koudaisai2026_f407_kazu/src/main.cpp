#include <Arduino.h>
#include "ps3_sbdbt.h"
#include "pin_define.h"

using namespace raven;

#define FAMIMA 0 // 起動音のONOFF
#define UNICORN     0

#define DAIHIHOU_POSITION 510
#define COALA_POSITION 460
#define POSITION_2nd 550// 2段目の位置ad値(524mm)
#define POSITION_1st 495 // 1段目の位置ad値(212mm)
#define ARM_UNDER_LIMIT 405 // アームの最下点
#define ARM_UPPER_LIMIT 640 // アームの最上点

#define HAND_RELEASE_SPEED  0.85f // ハンド開放強さ
#define HAND_HOLD_SPEED     0.65f // ハンド保持強さ

/* PS3 joystick limits defination: */
#define X_MAX 100
#define X_MIN 20
#define Y_MAX 100
#define Y_MIN 20

#define CTRL_GAIN 2.0f

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
//int hand_A = PE_11; // TIM1_CH2
//int hand_B = PE_9;  // TIM1_CH1
//int hand_A = PA_2; // TIM1_CH2
//int hand_B = PA_3;  // TIM1_CH1

int BACKLIGHT = PA_6;

int out_A1 = PC_7;  // TIM8_CH2
int out_A2 = PC_8;  // TIM8_CH3
int out_B1 = PC_6;  // TIM8_CH1
int out_B2 = PC_9;  // TIM8_CH4
int out_A3 = PE_13; // TIM1_CH3
int out_B3 = PE_14; // TIM1_CH4

int BZ = PB_15; // TIM12_CH2

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
HardwareTimer *toneTimer;// BZ用

int bpm=64;
int daihihou_flag = 0;  // 1にするとアーム位置を大秘宝位置に強制移動
int coala_flag = 0;  // 1にするとアーム位置を秘宝位置に強制移動
int jimen_flag = 0;  // 1にするとアーム位置を地面に強制移動
int now_vri = 0;  // VRの現在位置(AD)

unsigned char isYobikomi = 0;
unsigned char isCircle = 0;
unsigned char isOpen = 0, isClose = 0;  // アームの開閉状態
unsigned char isDaihihou = 0;  // 大秘宝の位置にアームを移動するか
unsigned char isCoala = 0;
unsigned char isUnder = 0;  // 地面すれすれに移動
unsigned char isStop = 0;
unsigned char is1st = 0; // 1段目212mm目標のad値
unsigned char is2nd = 0; // 2段目524mm目標のad値

float rx = 0.0f, ry = 0.0f, lx = 0.0f, ly = 0.0f;
float cosA = 0.0f,cosB = 0.0f,cosC = 0.0f;  // 各ホイールとの角度の比
float f1 = 0.0f, f2 = 0.0f, f3 = 0.0f;  // モータにかける最終pwm
float theta = 0.0f;  // 中心からのずれ
float ctrl_abs = 0.0f; // 方角(絶対値)
unsigned long cnt0 = 0, cnt1 = 0;
float duty0 = 0.0f;
float omega = 0.0f;
float omega_filtered = 0.0f; // 慣性フィルタ用
float straight_gain_x = 0.0f, straight_gain_y = 0.0f; // 直進補正用
float lx_filtered = 0.0f;
float ly_filtered = 0.0f;
static float lpf_state = 0.0f; // AD入力平均化用LPF
static float arm_cmd_filtered = 0.0f;
const int INTERVAL = 100;
int j = 0;
int hand_on = 0;
unsigned long cnt_h= 0;

volatile int handDuty = 0;   // 0〜100
volatile int handDir = 0; // -1:解放, 0:停止, 1:閉める


void motor1(float pwm );
void motor2(float pwm );
void motor3(float pwm );
void arm(float pwm );
void interrupt_005ms( void );
void setColor( int state_r, int state_g, int state_b );
void motorHand(float pwm);
float mapf(float x, float in_min, float in_max, float out_min, float out_max);
float expo(float x, float e);
void PwmArm(float pwm);
unsigned int readAnalogAveraged10bit(int pin);
void setPWMFrequency(float freq);
void ArmGoToPosition(int target_ad);
void famima( int bpm );
void yobikomi_No4(int bpm);
void toneSTM(float freq, int bpm, float note);

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


    // PWM
    pinMode(BACKLIGHT, OUTPUT);
    pinMode(out_A1, OUTPUT);
    pinMode(out_A2, OUTPUT);
    pinMode(out_B1, OUTPUT);
    pinMode(out_B2, OUTPUT);
    pinMode(out_A3, OUTPUT);
    pinMode(out_B3, OUTPUT);
    pinMode(arm_A, OUTPUT);
    pinMode(arm_B, OUTPUT);
    pinMode(hand_A, OUTPUT);
    pinMode(hand_B, OUTPUT);

    pinMode(RED, OUTPUT);
    pinMode(GREEN, OUTPUT);
    pinMode(BLUE, OUTPUT);

    // AnalogIn
    pinMode(VR_pin, INPUT_ANALOG);
    DebugSerial.begin(9600);
    DebugSerial.println("SBDBT driver started.");
    analogReadResolution(10); // analogReadの戻り値を10bitとする

    timer1 = new HardwareTimer(TIM4); // TIM4をタイマー割込みとして使う
    timer1->setOverflow(30000, HERTZ_FORMAT);  // 30kHz
    timer1->attachInterrupt(interrupt_005ms); // 割り込み関数を登録
    timer1->resume();  // タイマー開始

    // 足回りモータのpwm周期を125kHzに設定
    pwmTimer->setOverflow(125000, HERTZ_FORMAT);  // 125kHz
    pwmTimer->resume();
    pwm1->setOverflow(125000, HERTZ_FORMAT);  // 125kHz
    pwm1->resume();

    // アーム上下モータ用
    pwm2->setOverflow(125000, HERTZ_FORMAT);  // 125kHz
    pwm2->resume();

    // BZ用 TIM12_CH2はここで初期化
    __HAL_RCC_TIM12_CLK_ENABLE();
    toneTimer = new HardwareTimer(TIM12);
    toneTimer->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, PB15_ALT2);
    //toneTimer->setCaptureCompare(2, 50, PERCENT_COMPARE_FORMAT);
    toneTimer->pause();

    digitalWrite(RESET_LED, HIGH); // 準備完了
#if FAMIMA
    // famima(64); // ファミマの音
    yobikomi_No4(130);
#else
    toneSTM(4000, bpm, 0.07);
    toneSTM(0,    bpm, 0.07);
    toneSTM(4000, bpm, 0.07);
    toneSTM(0,    bpm, 0.07);

#endif  // FAMIMA

    cnt0 = 0;
    cnt1 = 0;
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

    isYobikomi  = sbdbt.L2();
    isCircle = sbdbt.L1();
    isOpen   = sbdbt.batu();
    isClose  = (sbdbt.maru() | sbdbt.R2());
    isDaihihou = sbdbt.ue();
    isCoala    = sbdbt.hidari();
    isUnder    = sbdbt.sita();
    is1st      = sbdbt.sankaku();    // 1段目目標
    is2nd      = sbdbt.sikaku();     // 2段目目標

    // LED色設定
    if(isYobikomi)       setColor(1,1,1);
    else if(is1st)       setColor(0,1,0);
    else if(is2nd)       setColor(1,0,1);
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
    lx_filtered = 0.85f * lx_filtered + 0.15f * lx;

    // 直進左右補正（左右移動時（lxの絶対値が大きい時）にlyを自動で弱める）
    straight_gain_y = 1.0f - fabs(lx);
    ly *= straight_gain_y;
    ly_filtered = 0.85f * ly_filtered + 0.15f * ly;
    theta = atan2(lx_filtered, ly_filtered);
    ctrl_abs = CTRL_GAIN * sqrt(lx_filtered*lx_filtered + ly_filtered*ly_filtered);

    cosA = ctrl_abs * cos( 30 * M_PI / 180 - theta);
    cosB = ctrl_abs * cos(150 * M_PI / 180 - theta);
    cosC = ctrl_abs * cos(270 * M_PI / 180 - theta);

    f1 = mapf(cosA, -1.42f, 1.42f,  1.0f, -1.0f);
    f2 = mapf(cosB, -1.42f, 1.42f, -1.0f,  1.0f);
    f3 = mapf(cosC, -1.42f, 1.42f, -1.0f,  1.0f);
 
    // 右スティックで回転操作と組み合わせる
    omega = rx;
    omega_filtered = 0.85f * omega_filtered + 0.15f * omega; // 慣性フィルタ
    f1 += -1.0f*omega_filtered;
    f2 += omega_filtered;
    f3 += omega_filtered;

    daihihou_flag = isDaihihou ? 1 : 0;
    coala_flag    = isCoala    ? 1 : 0;
    jimen_flag    = isUnder    ? 1 : 0;
    
    if(isYobikomi){
        motor1(0);
        motor2(0);
        motor3(0);
        PwmArm(0);
        motorHand(0);
        yobikomi_No4(130);
        isYobikomi = 0;
    }


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

    // ハンド開閉の判定
    // 実際のモータ動作は割り込み内部で行う。motorHandはPwmの計算のみ。
    // ハンド開閉の判定

    if (isOpen == 1 && isClose == 0) {
        // アーム開放
        motorHand(HAND_RELEASE_SPEED);
    }
    else if (isOpen == 0 && isClose == 1) {
        // アーム保持
        motorHand(-HAND_HOLD_SPEED);
    }
    else {
        isOpen = 0;
        isClose = 0;
        motorHand(0.0f);
    }

    // モーター動作
    motor1(f1);
    motor2(f2);
    motor3(-1.0f * f3);

    // アーム動作
    if (isCoala) {
        ArmGoToPosition(COALA_POSITION);  // 三角ボタン押下中はコアラ位置へ移動
    } 
    else if(isDaihihou){
        ArmGoToPosition(DAIHIHOU_POSITION); // 四角ボタン押下中は大秘宝の位置へ移動
    }
    else if(isUnder){
        ArmGoToPosition(ARM_UNDER_LIMIT); // 下↓で最下へ移動
    }
    else if(is1st){
        ArmGoToPosition(POSITION_1st); // 左キー押し下げ中は1段目へ移動
    }
    else if(is2nd){
        ArmGoToPosition(POSITION_2nd); // 上キー押し下げ中は2段目へ移動
    }
    else {
        PwmArm(ry); // 通常のジョイスティック操作
    }

}

void interrupt_005ms(void)
{
    cnt1++;
    if (cnt1 >= 100) {
        cnt1 = 0;
    }

    // 停止
    if (handDuty == 0 || handDir == 0) {
        digitalWrite(hand_A, LOW);
        digitalWrite(hand_B, LOW);
    }

    // PWM ON
    else if (cnt1 < handDuty) {
        if (handDir > 0) {
            // 正転
            digitalWrite(hand_A, HIGH);
            digitalWrite(hand_B, LOW);
        }
        else {
            // 逆転
            digitalWrite(hand_A, LOW);
            digitalWrite(hand_B, HIGH);
        }
    }
    // PWM OFF
    else {
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

void PwmArm(float pwm)
{
    const float DEADZONE    = 0.08f; // この範囲内はスティック中央判定
    const float EXPO_FACTOR = 0.50f; // スティック中央の指数関数補正
    const float LPF_ALPHA   = 0.30f; // ローパスフィルタ係数：小さいほど滑らか、大きいほど機敏
    const float STOP_EPS    = 0.05f; // このコマンド値未満なら完全停止判定

    float duty;
    float input = pwm;
    if (fabs(input) < DEADZONE) input = 0.0f;
    if (input == 0.0f || fabs(input) < STOP_EPS) {
        analogWrite(arm_A, 0);
        analogWrite(arm_B, 0);
        return;
    }
   

    if (pwm == 0.0f) {
        duty = 0.5f;
        } else {
            duty = mapf(pwm, -1.0f, 1.0f, 0.0f, 1.0f);
             // 機構下限に到達
            if(now_vri < ARM_UNDER_LIMIT){
                if(duty>0.5f){
                    pwm=0.0f;
                    analogWrite(arm_A, 0);
                    analogWrite(arm_B, 0);
                    return;  
                }    
            }
            // 機構上限に到達
            if(now_vri > ARM_UPPER_LIMIT){
                if(duty<0.5f){
                    pwm=0.0f;
                    analogWrite(arm_A, 0);
                    analogWrite(arm_B, 0);
                    return;  
                }
            }
    }

    if (duty >= 1.0f) duty = 0.95f;
    if (duty <= 0.0f) duty = 0.01f;

    analogWrite(arm_B, duty * 255);
    analogWrite(arm_A, (1.0f - duty) * 255);
}

// アームをPWMで動かす。入力はジョイスティックry座標
// 範囲: -1で逆転: 0で停止: 1で正転
void motorHand(float pwm)
{
    float duty;

    if (pwm == 0.0f) {
        handDuty = 0;
        handDir = 0;
    }
    else {

        // 回転方向
        if (pwm > 0.0f) {
            handDir = 1;
        }
        else {
            handDir = -1;
        }

        duty = mapf(fabs(pwm), 0.0f, 1.0f, 0.0f, 1.0f);

        if (duty >= 1.0f) duty = 0.95f;
        if (duty <= 0.0f) duty = 0.05f;

        handDuty = (int)(duty * 100);
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

    for(int i = 0; i < SAMPLE_COUNT; i++) {
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
    if(lpf_state < 0.0f)   lpf_state = 0.0f;
    if(lpf_state > 1023.0f) lpf_state = 1023.0f;

    return (unsigned int)(lpf_state + 0.5f);
}

// VR(AD値)フィードバックでアームを目標位置へ移動する
void ArmGoToPosition(int target_ad)
{
    // ===== 調整パラメータ =====
    const float VR_ALPHA      = 0.15f;  // VR平滑化係数
    const float POS_GAIN_FAR  = 0.008f; // 目標から遠いときのゲイン
    const float POS_GAIN_NEAR = 0.004f; // 目標付近のゲイン
    const float MAX_SPEED     = 0.35f;  // 最大速度

    // 停止判定のヒステリシス
    const int STOP_ENTER      = 10;     // この誤差以内に入ったら停止
    const int STOP_EXIT       = 25;     // 停止中はこれを超えるまで再始動しない

    // 目標付近として扱う範囲
    const int NEAR_RANGE      = 60;


    // ===== VR値をローパスフィルタ =====
    static float vr_filtered = 0.0f;
    static bool initialized = false;

    if (!initialized) {
        vr_filtered = analogRead(VR_pin);
        initialized = true;
    }

    int vr_raw = analogRead(VR_pin);

    vr_filtered =
        (1.0f - VR_ALPHA) * vr_filtered
        + VR_ALPHA * (float)vr_raw;

    now_vri = (int)vr_filtered;


    // ===== 目標値が変わったら停止状態を解除 =====
    static int last_target = target_ad;
    static bool holding = false;

    if (target_ad != last_target) {
        holding = false;
        last_target = target_ad;
    }


    // ===== 誤差 =====
    int error = now_vri - target_ad;
    int abs_error = abs(error);


    // ===== 停止中 =====
    if (holding) {

        // 目標から十分離れたら再始動
        if (abs_error > STOP_EXIT) {
            holding = false;
        }
        else {
            analogWrite(arm_A, 0);
            analogWrite(arm_B, 0);
            arm_cmd_filtered = 0.0f;
            return;
        }
    }


    // ===== 目標位置に到達 =====
    if (abs_error <= STOP_ENTER) {

        holding = true;

        analogWrite(arm_A, 0);
        analogWrite(arm_B, 0);
        arm_cmd_filtered = 0.0f;

        return;
    }

    float gain;

    if (abs_error > NEAR_RANGE) {
        gain = POS_GAIN_FAR;
    }
    else {
        gain = POS_GAIN_NEAR;
    }

    float cmd = gain * (float)error;
    if (cmd > MAX_SPEED)  cmd = MAX_SPEED;
    if (cmd < -MAX_SPEED) cmd = -MAX_SPEED;
    PwmArm(cmd);
}

void setPWMFrequency(float freq) {
    if(freq <= 0) return;
    toneTimer->setOverflow((uint32_t)freq, HERTZ_FORMAT);
}

void toneSTM(float freq, int bpm, float note)
{
    // 音符の長さ(ms)
    unsigned long duration_ms = (unsigned long)(note * (60000.0f / bpm));

    if(freq > 0){
        // 周波数設定
        setPWMFrequency(freq);
        toneTimer->setCaptureCompare(2, 50, PERCENT_COMPARE_FORMAT);
        toneTimer->resume();   // PWM ON
    } else {
        // 無音
        toneTimer->setCaptureCompare(2, 0, PERCENT_COMPARE_FORMAT);
        toneTimer->pause();    // PWM OFF
    }
    delay(duration_ms);

    // 音を止める（休符）
    toneTimer->setCaptureCompare(2, 0, PERCENT_COMPARE_FORMAT);
    toneTimer->pause();
}

void famima(int bpm){
    // ファミマの音
    toneSTM(740, bpm, 0.25);
    toneSTM(587, bpm, 0.25);
    toneSTM(440, bpm, 0.25);
    toneSTM(587, bpm, 0.25);
    toneSTM(659, bpm, 0.25);
    toneSTM(880, bpm, 0.5);
    toneSTM(0,   bpm, 0.25);
    toneSTM(659, bpm, 0.25);
    toneSTM(740, bpm, 0.25);
    toneSTM(659, bpm, 0.25);
    toneSTM(440, bpm, 0.25);
    toneSTM(587, bpm, 0.5);
    toneSTM(0,   bpm, 0.1);
}

void yobikomi_No4(int bpm) {

    const int D  = 587;
    const int E  = 659;
    const int Fs = 740;
    const int G  = 784;
    const int A  = 880;
    const int B  = 988;

    const float GAP = 0.20f;

    // 1小節目
    toneSTM(A,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(A,  bpm, 1.0f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(B,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(A,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(Fs, bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(A,  bpm, 1.0f - GAP);
    toneSTM(0,       bpm, GAP);

    // 2小節目
    toneSTM(A,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(A,  bpm, 1.0f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(B,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(A,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(Fs, bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(A,  bpm, 1.0f - GAP);
    toneSTM(0,       bpm, GAP);

    // 3小節目
    toneSTM(D,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(D,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(D,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(E,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(Fs, bpm, 1.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(D,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    // 4小節目
    toneSTM(Fs, bpm, 1.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(A,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(A,  bpm, 2.0f - GAP);
    toneSTM(0,       bpm, GAP);

    // 5小節目
    toneSTM(D,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(D,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(D,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(E,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(Fs, bpm, 2.0f - GAP);
    toneSTM(0,       bpm, GAP);

    // 6小節目
    toneSTM(D,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(D,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(D,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(E,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(Fs, bpm, 2.0f - GAP);
    toneSTM(0,       bpm, GAP);

    // 7小節目
    toneSTM(E,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(E,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(E,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(D,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(E,  bpm, 1.0f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(Fs, bpm, 1.0f - GAP);
    toneSTM(0,       bpm, GAP);

    // 8小節目
    toneSTM(A,  bpm, 1.0f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(G,  bpm, 1.0f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(Fs, bpm, 1.0f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(E,  bpm, 1.0f - GAP);
    toneSTM(0,       bpm, GAP);

    // 9小節目
    toneSTM(A,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(A,  bpm, 1.0f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(B,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(A,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(Fs, bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(A,  bpm, 1.0f - GAP);
    toneSTM(0,       bpm, GAP);

    // 10小節目
    toneSTM(A,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(A,  bpm, 1.0f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(B,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(A,  bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(Fs, bpm, 0.5f - GAP);
    toneSTM(0,       bpm, GAP);

    toneSTM(E,  bpm, 1.0f - GAP);
    toneSTM(0,       bpm, GAP);

    // 11小節目
    toneSTM(D, bpm, 4.0f);

    // 最後に消灯
    setColor(LOW, LOW, LOW);
}
