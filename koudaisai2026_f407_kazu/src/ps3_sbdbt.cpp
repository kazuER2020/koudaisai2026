#include <Arduino.h>
#include "ps3_sbdbt.h"

namespace raven {

// ======================================================
// 既存：HardwareSerial を参照で受け取るコンストラクタ
// ======================================================
SBDBT::SBDBT(HardwareSerial &serial, uint32_t baud)
{
    i = 0;
    readable = 0;
    position = 0;

    sr = &serial;
    sr->begin(baud);

    for (uint8_t k = 0; k < length; ++k) {
        tmp[k]  = 0;
        data[k] = 0;
    }
}

// ======================================================
// ★追加：ピン番号を指定して HardwareSerial を内部生成するコンストラクタ
// ======================================================
SBDBT::SBDBT(uint32_t rx, uint32_t tx, uint32_t baud)
{
    i = 0;
    readable = 0;
    position = 0;

    // HardwareSerial(rx, tx) は uint32_t を受け取る
    sr = new HardwareSerial(rx, tx);
    sr->begin(baud);

    for (uint8_t k = 0; k < length; ++k) {
        tmp[k]  = 0;
        data[k] = 0;
    }
}

// ======================================================
// シリアル受信処理（loop() から呼ぶ）
// ======================================================
void SBDBT::update()
{
    while (sr->available()) {
        processByte(sr->read());
    }
}

// ======================================================
// 1バイト処理（mbed版 getf() の移植）
// ======================================================
void SBDBT::processByte(char ch)
{
    tmp[i] = ch;

    if (tmp[i] == value) {              // 訂正用定数 128
        if (i != position) {
            tmp[position] = value;
            i = position + 1;
        } else {
            i++;
        }
    } else {
        if (i != position) {
            i++;
        }
    }

    if (i >= length) {
        for (uint8_t k = 0; k < length; ++k) {
            data[k] = tmp[k];
        }
        readable = 1;
        i = 0;
    }
}

// ======================================================
// デバッグ出力
// ======================================================
void SBDBT::print(Stream &out)
{
    out.printf("%3d %3d %3d %3d %3d %3d %3d %3d\n",
               data[0], data[1], data[2], data[3],
               data[4], data[5], data[6], data[7]);
}

// ======================================================
// ボタン判定（mbed版と同じ）
// ======================================================
char SBDBT::maru()     { return (data[2] & 64) ? 1 : 0; }
char SBDBT::batu()     { return (data[2] & 32) ? 1 : 0; }
char SBDBT::sikaku()   { return (data[1] & 1)  ? 1 : 0; }
char SBDBT::sankaku()  { return (data[2] & 16) ? 1 : 0; }

char SBDBT::L1()       { return (data[1] & 2)  ? 1 : 0; }
char SBDBT::L2()       { return (data[1] & 4)  ? 1 : 0; }
char SBDBT::R1()       { return (data[1] & 8)  ? 1 : 0; }
char SBDBT::R2()       { return (data[1] & 16) ? 1 : 0; }

char SBDBT::ue()       { return (data[2] & 1)  ? 1 : 0; }
char SBDBT::sita()     { return (data[2] & 2)  ? 1 : 0; }
char SBDBT::migi()     { return (data[2] & 4)  ? 1 : 0; }
char SBDBT::hidari()   { return (data[2] & 8)  ? 1 : 0; }

// ======================================================
// アナログスティック
// ======================================================
signed char SBDBT::rs_x() { return data[5]; }
signed char SBDBT::rs_y() { return data[6]; }
signed char SBDBT::ls_x() { return data[3]; }
signed char SBDBT::ls_y() { return data[4]; }

} // namespace raven
