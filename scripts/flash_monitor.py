import sys
import os
import subprocess

import serial
import serial.tools.list_ports

from bullet import Bullet, SlidePrompt, Check, Input, YesNo, Numbers
from bullet import styles
from bullet import colors

device = None
BIN_PATH = "STM32_Programmer_CLI"

SCRIPTS_DIR = os.path.dirname(os.path.realpath(__file__))
ROOT_DIR = os.path.join(SCRIPTS_DIR, os.pardir)
BOOTLOADER_BUILD_PATH = os.path.join(ROOT_DIR, "build/firmware/bootloader")
CONTROLLERS_BUILD_PATH = os.path.join(ROOT_DIR, "build/firmware/controllers")
FIRMWARE_PATHS = {
    "RC": os.path.join(CONTROLLERS_BUILD_PATH, "RC/RC.elf"),
    "FC": os.path.join(CONTROLLERS_BUILD_PATH, "FC/FC.elf"),
    "ACM": os.path.join(CONTROLLERS_BUILD_PATH, "ACM/ACM.elf"),
    "WDAQ TX": os.path.join(CONTROLLERS_BUILD_PATH, "WDAQ/TX/STM32/WDAQ_TX.elf"),
    "WDAQ RX": os.path.join(CONTROLLERS_BUILD_PATH, "WDAQ/RX/STM32/WDAQ_RX.elf"),
    "BOOTLOADER": os.path.join(BOOTLOADER_BUILD_PATH, "BOOTLOADER_STM32G474.elf"),
    "RC_TEST": os.path.join(CONTROLLERS_BUILD_PATH, "RC/tests/self/test_self.elf"),
    "ACM_TEST": os.path.join(CONTROLLERS_BUILD_PATH, "ACM/tests/bms/test_bms_acm.elf"),
    "H7DEV": os.path.join(CONTROLLERS_BUILD_PATH, "H7dev/H7dev.elf"),
    "BOOTLOADER_H755": os.path.join(BOOTLOADER_BUILD_PATH, "BOOTLOADER_STM32H755.elf"),
}
BOOTLOADER_PATH = os.path.join(ROOT_DIR, "build/firmware/bootloader/BOOTLOADER.elf")


def get_devices():
    return list(map(lambda dev: dev.device, serial.tools.list_ports.comports()))


def get_prompts():
    global device
    prompts = []
    devices = get_devices()
    if len(devices) == 0:
        print("no devices found")
        sys.exit(1)
    elif len(devices) == 1:
        device = devices[0]
    else:
        prompts.append(
            Bullet(
                "What device? ",
                choices=devices,
                bullet=" >",
                margin=2,
                bullet_color=colors.bright(colors.foreground["cyan"]),
                background_color=colors.background["black"],
                background_on_switch=colors.background["cyan"],
                word_color=colors.foreground["white"],
                word_on_switch=colors.foreground["white"],
            )
        )
    prompts.append(
        Bullet(
            "What board? ",
            choices=[
                "RC",
                "FC",
                "ACM",
                "WDAQ TX",
                "WDAQ RX",
                "BOOTLOADER",
                "RC_TEST",
                "ACM_TEST",
                "H7DEV",
                "BOOTLOADER_H755",
            ],
            bullet=" >",
            margin=2,
            bullet_color=colors.bright(colors.foreground["cyan"]),
            background_color=colors.background["black"],
            background_on_switch=colors.background["cyan"],
            word_color=colors.foreground["white"],
            word_on_switch=colors.foreground["white"],
        )
    )
    prompts.append(
        YesNo("Flash bootloader? ", word_color=colors.foreground["yellow"], default="n")
    )
    prompts.append(
        YesNo("Monitor? ", word_color=colors.foreground["yellow"], default="n")
    )
    return prompts


def flash(board, bootloader):
    global device
    if bootloader:
        subprocess.run(
            [
                "STM32_Programmer_CLI",
                "-c",
                # f"port=SWD",
                f"port={device}",
                "-w",
                BOOTLOADER_PATH,
                # "--verify",
            ],
            timeout=120,
        )
    subprocess.run(
        [
            "STM32_Programmer_CLI",
            "-c",
            # f"port=SWD",
            f"port={device}",
            "-w",
            FIRMWARE_PATHS[board],
            # "--verify",
            "--go",
        ],
        timeout=120,
    )


def monitor(port, baudrate=112500):
    print("monitoring...")

    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        parity=serial.PARITY_ODD,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.SEVENBITS,
    )

    while True:
        line = ser.readline().strip()
        line = line.decode("utf-8")
        line = line.split("\r")
        print(line[0])

    ser.close()


def prompt_flash():
    global device
    do_monitor = False
    prompts = get_prompts()
    cli = SlidePrompt(prompts)
    results = cli.launch()
    if device is None:
        device = results[0][1]
        board = results[1][1]
        bootloader = results[2][1]
        do_monitor = results[3][1]
    else:
        board = results[0][1]
        bootloader = results[1][1]
        do_monitor = results[2][1]
    return board, bootloader, do_monitor


def prompt_monitor():
    global device
    cli = SlidePrompt(
        [
            Bullet(
                "What device? ",
                choices=get_devices(),
                bullet=" >",
                margin=2,
                bullet_color=colors.bright(colors.foreground["cyan"]),
                background_color=colors.background["black"],
                background_on_switch=colors.background["cyan"],
                word_color=colors.foreground["white"],
                word_on_switch=colors.foreground["white"],
            )
        ]
    )
    results = cli.launch()
    device = results[0][1]


if __name__ == "__main__":
    do_flash = False
    do_monitor = False
    if len(sys.argv) == 2:
        if sys.argv[1] == "flash":
            do_flash = True
        elif sys.argv[1] == "monitor":
            do_monitor = True
    if do_flash:
        board, bootloader, do_monitor = prompt_flash()
        flash(board, bootloader)
        if do_monitor:
            monitor(device)
    elif do_monitor:
        prompt_monitor()
        monitor(device)