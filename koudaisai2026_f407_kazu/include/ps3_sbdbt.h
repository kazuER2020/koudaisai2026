#pragma once
#include <Arduino.h>

namespace raven {

class SBDBT {
public:
    // 既存：HardwareSerial を参照で受け取る版
    SBDBT(HardwareSerial &serial, uint32_t baud = 2400);

    // ★追加：ピン番号を指定して HardwareSerial を内部生成する版
    SBDBT(uint32_t rx, uint32_t tx, uint32_t baud = 2400);


    void update();
    bool available() const { return readable == 1; }
    void print(Stream &out);

    // ボタン系
    char maru();
    char batu();
    char sikaku();
    char sankaku();
    char L1();
    char L2();
    char R1();
    char R2();
    char ue();
    char sita();
    char migi();
    char hidari();

    // スティック
    signed char rs_x();
    signed char rs_y();
    signed char ls_x();
    signed char ls_y();

private:
    HardwareSerial* sr;   // ★内部で new する可能性があるのでポインタに変更

    uint8_t i;
    uint8_t readable;
    uint8_t position;

    static const uint8_t length = 8;
    static const char value = 128;

    char tmp[length];
    char data[length];

    void processByte(char ch);
};

} // namespace raven
