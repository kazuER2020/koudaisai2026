# raspberry pi側のコード
# USBシリアルで受けたコードを画像表示+音声再生に対応させる
#!/usr/bin/python
# -*- coding: utf-8 -*-

import serial
import time
import sys
import os
import threading
import traceback

os.environ['SDL_AUDIODRIVER'] = 'alsa'

import pygame
from pygame.locals import *

WATCHDOG_TIMEOUT = 5.0
AUDIO_ENABLE = False

last_receive_time = time.monotonic()

received_num = 0
old_received_num = -1
isPlayingOneshot = 0

latest_received_num = None
latest_lock = threading.Lock()

screen = None

image_cache = {}
current_image = None

recv_thread = None


def restart_program(reason="unknown"):
    print("")
    print("===================================")
    print("RESTART")
    print(reason)
    print("===================================")
    print("")

    try:
        pygame.quit()
    except:
        pass

    os.execv(
        sys.executable,
        [sys.executable] + sys.argv
    )


def playEffects(effect_number, volume=0.5):
    global AUDIO_ENABLE
    if not AUDIO_ENABLE:
        return

    effect_dict = {
        1: "motion_moving.wav",
        2: "motion_arm.wav",
        3: "hand_close.wav",
        4: "hand_open.wav",
        5: "yobikomikun.wav",
        6: "usa_ha_1.wav",
        7: "usa_yeah_1.wav",
        8: "uc.wav",
        9: "winxp_start.wav",
        10: "winxp_shutdown.wav"
    }

    if effect_number in [0, -1]:
        pygame.mixer.music.stop()
        return

    if effect_number not in effect_dict:
        print("effects not found.")
        return

    filename = effect_dict[effect_number]

    print(f'num:{effect_number}, filename:{filename}, vol:{volume}')
    pygame.mixer.music.stop()
    pygame.mixer.music.load(filename)
    pygame.mixer.music.set_volume(volume)
    pygame.mixer.music.play()

def photoShow(filename):
    global current_image
    if current_image == filename:
        return

    current_image = filename
    img = image_cache[filename]
    rect_img = img.get_rect()
    rect_img.center = screen.get_rect().center
    screen.blit(img, rect_img)
    pygame.display.flip()
    pygame.event.pump()


def serial_receive_worker(ser):

    global latest_received_num
    global last_receive_time

    while ser.is_open:
        try:
            raw = ser.readline()
            if not raw:
                continue

            try:
                data = raw.decode('utf-8').strip()
                value = int(data)
                with latest_lock:
                    latest_received_num = value
                last_receive_time = time.monotonic()

            except ValueError:
                print(f'変換失敗:{raw}')

        except Exception as e:
            print(f"serial error:{e}")
            break


def watchdog_worker():
    global last_receive_time
    global recv_thread

    while True:

        try:

            elapsed = time.monotonic() - last_receive_time

            if elapsed > WATCHDOG_TIMEOUT:

                restart_program(
                    f"STM timeout ({elapsed:.1f}s)"
                )

            if recv_thread is not None:

                if not recv_thread.is_alive():

                    restart_program(
                        "receive thread dead"
                    )

            time.sleep(0.5)

        except Exception:

            traceback.print_exc()

            restart_program("watchdog exception")


def decode_STM_serial(num, state_changed):

    global isPlayingOneshot
    global old_received_num

    if not isinstance(num, (int, float)):
        print('not Integer.')
        return

    if num & 0x01:

        if state_changed:
            photoShow("moving.png")

        if not pygame.mixer.music.get_busy():
            playEffects(1)

    elif num & 0x02:

        if state_changed:
            photoShow("arm.png")

        if not pygame.mixer.music.get_busy():
            playEffects(2)

    elif num & 0x04:

        photoShow("hand_close.png")

        if isPlayingOneshot == 0:
            playEffects(3)
            isPlayingOneshot = 1

    elif num & 0x08:

        photoShow("hand_open.png")

        if isPlayingOneshot == 0:
            playEffects(4)
            isPlayingOneshot = 1

    elif num & 0x10:

        if old_received_num != num:
            playEffects(5)

    elif num == 0x00:

        if state_changed:
            photoShow("standby.png")

        isPlayingOneshot = 0


def load_images():

    files = [
        "standby.png",
        "moving.png",
        "arm.png",
        "hand_close.png",
        "hand_open.png"
    ]

    for file in files:

        img = pygame.image.load(file).convert()

        img = pygame.transform.scale(
            img,
            (800, 480)
        )

        image_cache[file] = img


def connect_serial():

    while True:

        try:

            ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.05 )
            print("STM connected")
            print(ser.portstr)
            return ser

        except Exception as e:
            print("STM waiting...")
            print(e)
            time.sleep(1)


def main():
    global screen
    global old_received_num
    global recv_thread
    global last_receive_time
    global AUDIO_ENABLE
    pygame.mixer.pre_init(44100, -16, 2, 4096) # settings for ".wav"
    pygame.init()
    try:
        pygame.mixer.init()
        AUDIO_ENABLE = True
        print("audio init success")

    except Exception as e:
        AUDIO_ENABLE = False
        print("audio init failed")
        print(e)

    screen = pygame.display.set_mode((800, 480),pygame.NOFRAME)

    pygame.mouse.set_visible(False)
    pygame.display.set_caption("Photo")
    load_images()
    print(os.getcwd())
    ser = connect_serial()
    last_receive_time = time.monotonic()
    photoShow("standby.png")
    playEffects(9)

    recv_thread = threading.Thread(
        target=serial_receive_worker,
        args=(ser,),
        daemon=True
    )

    recv_thread.start()

    wd_thread = threading.Thread(
        target=watchdog_worker,
        daemon=True
    )

    wd_thread.start()

    while True:
        try:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt

            with latest_lock:
                current = latest_received_num

            if current is not None:
                decode_STM_serial(current, state_changed=(current != old_received_num))
                old_received_num = current

            time.sleep(0.005)

        except KeyboardInterrupt:
            print("終了します")

            if ser.is_open:
                playEffects(10)
                time.sleep(3)
                ser.close()
            pygame.quit()

            return

        except Exception:
            traceback.print_exc()
            restart_program("main loop exception")


if __name__ == '__main__':
    main()
