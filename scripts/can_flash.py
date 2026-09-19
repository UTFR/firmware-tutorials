#!/usr/bin/env python3
"""
DFU sender for STM32 CAN bootloader.
Matches the protocol expected by dfu.c.

Requires: pip install python-can crc
Hardware: PEAK PCAN-USB
"""

import argparse
import platform
import struct
import subprocess
import sys
import time
from pathlib import Path
from enum import IntEnum

import can

from elftools.elf.elffile import ELFFile


class Controller(IntEnum):
    ACM = 1
    FC = 2
    RC = 3
    WDAQ = 4


CONTROLLER_STR_MAP = {
    "ACM": Controller.ACM,
    "FC": Controller.FC,
    "RC": Controller.RC,
    "WDAQ": Controller.WDAQ,
}

FLASH_BASE = 0x08000000
FLASH_END = 0x08000000 + 512 * 1024

CMD_ACK = 0x100
CMD_NACK = 0x101
CMD_IMAGE_HEADER = 0x102
CMD_FIRMWARE_DATA = 0x103
CMD_FIRMWARE_DATA_FINISH = 0x104
CMD_REQUEST_DFU = 0x105

BLOCK_SIZE = 2048
CHUNK_SIZE = 8
ACK_TIMEOUT_S = 10.0
MAX_BLOCK_RETRIES = 3

_CRC32_TAB = [
    0x00000000,
    0x77073096,
    0xEE0E612C,
    0x990951BA,
    0x076DC419,
    0x706AF48F,
    0xE963A535,
    0x9E6495A3,
    0x0EDB8832,
    0x79DCB8A4,
    0xE0D5E91E,
    0x97D2D988,
    0x09B64C2B,
    0x7EB17CBD,
    0xE7B82D07,
    0x90BF1D91,
    0x1DB71064,
    0x6AB020F2,
    0xF3B97148,
    0x84BE41DE,
    0x1ADAD47D,
    0x6DDDE4EB,
    0xF4D4B551,
    0x83D385C7,
    0x136C9856,
    0x646BA8C0,
    0xFD62F97A,
    0x8A65C9EC,
    0x14015C4F,
    0x63066CD9,
    0xFA0F3D63,
    0x8D080DF5,
    0x3B6E20C8,
    0x4C69105E,
    0xD56041E4,
    0xA2677172,
    0x3C03E4D1,
    0x4B04D447,
    0xD20D85FD,
    0xA50AB56B,
    0x35B5A8FA,
    0x42B2986C,
    0xDBBBC9D6,
    0xACBCF940,
    0x32D86CE3,
    0x45DF5C75,
    0xDCD60DCF,
    0xABD13D59,
    0x26D930AC,
    0x51DE003A,
    0xC8D75180,
    0xBFD06116,
    0x21B4F4B5,
    0x56B3C423,
    0xCFBA9599,
    0xB8BDA50F,
    0x2802B89E,
    0x5F058808,
    0xC60CD9B2,
    0xB10BE924,
    0x2F6F7C87,
    0x58684C11,
    0xC1611DAB,
    0xB6662D3D,
    0x76DC4190,
    0x01DB7106,
    0x98D220BC,
    0xEFD5102A,
    0x71B18589,
    0x06B6B51F,
    0x9FBFE4A5,
    0xE8B8D433,
    0x7807C9A2,
    0x0F00F934,
    0x9609A88E,
    0xE10E9818,
    0x7F6A0DBB,
    0x086D3D2D,
    0x91646C97,
    0xE6635C01,
    0x6B6B51F4,
    0x1C6C6162,
    0x856530D8,
    0xF262004E,
    0x6C0695ED,
    0x1B01A57B,
    0x8208F4C1,
    0xF50FC457,
    0x65B0D9C6,
    0x12B7E950,
    0x8BBEB8EA,
    0xFCB9887C,
    0x62DD1DDF,
    0x15DA2D49,
    0x8CD37CF3,
    0xFBD44C65,
    0x4DB26158,
    0x3AB551CE,
    0xA3BC0074,
    0xD4BB30E2,
    0x4ADFA541,
    0x3DD895D7,
    0xA4D1C46D,
    0xD3D6F4FB,
    0x4369E96A,
    0x346ED9FC,
    0xAD678846,
    0xDA60B8D0,
    0x44042D73,
    0x33031DE5,
    0xAA0A4C5F,
    0xDD0D7CC9,
    0x5005713C,
    0x270241AA,
    0xBE0B1010,
    0xC90C2086,
    0x5768B525,
    0x206F85B3,
    0xB966D409,
    0xCE61E49F,
    0x5EDEF90E,
    0x29D9C998,
    0xB0D09822,
    0xC7D7A8B4,
    0x59B33D17,
    0x2EB40D81,
    0xB7BD5C3B,
    0xC0BA6CAD,
    0xEDB88320,
    0x9ABFB3B6,
    0x03B6E20C,
    0x74B1D29A,
    0xEAD54739,
    0x9DD277AF,
    0x04DB2615,
    0x73DC1683,
    0xE3630B12,
    0x94643B84,
    0x0D6D6A3E,
    0x7A6A5AA8,
    0xE40ECF0B,
    0x9309FF9D,
    0x0A00AE27,
    0x7D079EB1,
    0xF00F9344,
    0x8708A3D2,
    0x1E01F268,
    0x6906C2FE,
    0xF762575D,
    0x806567CB,
    0x196C3671,
    0x6E6B06E7,
    0xFED41B76,
    0x89D32BE0,
    0x10DA7A5A,
    0x67DD4ACC,
    0xF9B9DF6F,
    0x8EBEEFF9,
    0x17B7BE43,
    0x60B08ED5,
    0xD6D6A3E8,
    0xA1D1937E,
    0x38D8C2C4,
    0x4FDFF252,
    0xD1BB67F1,
    0xA6BC5767,
    0x3FB506DD,
    0x48B2364B,
    0xD80D2BDA,
    0xAF0A1B4C,
    0x36034AF6,
    0x41047A60,
    0xDF60EFC3,
    0xA867DF55,
    0x316E8EEF,
    0x4669BE79,
    0xCB61B38C,
    0xBC66831A,
    0x256FD2A0,
    0x5268E236,
    0xCC0C7795,
    0xBB0B4703,
    0x220216B9,
    0x5505262F,
    0xC5BA3BBE,
    0xB2BD0B28,
    0x2BB45A92,
    0x5CB36A04,
    0xC2D7FFA7,
    0xB5D0CF31,
    0x2CD99E8B,
    0x5BDEAE1D,
    0x9B64C2B0,
    0xEC63F226,
    0x756AA39C,
    0x026D930A,
    0x9C0906A9,
    0xEB0E363F,
    0x72076785,
    0x05005713,
    0x95BF4A82,
    0xE2B87A14,
    0x7BB12BAE,
    0x0CB61B38,
    0x92D28E9B,
    0xE5D5BE0D,
    0x7CDCEFB7,
    0x0BDBDF21,
    0x86D3D2D4,
    0xF1D4E242,
    0x68DDB3F8,
    0x1FDA836E,
    0x81BE16CD,
    0xF6B9265B,
    0x6FB077E1,
    0x18B74777,
    0x88085AE6,
    0xFF0F6A70,
    0x66063BCA,
    0x11010B5C,
    0x8F659EFF,
    0xF862AE69,
    0x616BFFD3,
    0x166CCF45,
    0xA00AE278,
    0xD70DD2EE,
    0x4E048354,
    0x3903B3C2,
    0xA7672661,
    0xD06016F7,
    0x4969474D,
    0x3E6E77DB,
    0xAED16A4A,
    0xD9D65ADC,
    0x40DF0B66,
    0x37D83BF0,
    0xA9BCAE53,
    0xDEBB9EC5,
    0x47B2CF7F,
    0x30B5FFE9,
    0xBDBDF21C,
    0xCABAC28A,
    0x53B39330,
    0x24B4A3A6,
    0xBAD03605,
    0xCDD70693,
    0x54DE5729,
    0x23D967BF,
    0xB3667A2E,
    0xC4614AB8,
    0x5D681B02,
    0x2A6F2B94,
    0xB40BBE37,
    0xC30C8EA1,
    0x5A05DF1B,
    0x2D02EF8D,
]


def load_elf_segments(path: Path):
    segments: list[tuple[int, bytes]] = []

    with path.open("rb") as f:
        elf = ELFFile(f)

        for seg in elf.iter_segments():
            if seg["p_type"] != "PT_LOAD":
                continue

            addr = seg["p_paddr"]
            data = seg.data()

            if len(data) == 0:
                continue

            # Filter only FLASH region
            if not (FLASH_BASE <= addr < FLASH_END):
                continue

            print(f"ELF segment: addr=0x{addr:08X}, size={len(data)} bytes")

            segments.append((addr, data))

    return segments


# def crc32(data: bytes) -> int:
#     import binascii

#     return binascii.crc32(data) & 0xFFFFFFFF


def crc32(data: bytes) -> int:
    """CRC-32 matching the C implementation in dfu.c: crc32(0, data, size)."""
    crc = 0 ^ 0xFFFFFFFF  # crc ^ ~0U
    for byte in data:
        crc = _CRC32_TAB[(crc ^ byte) & 0xFF] ^ (crc >> 8)
    return (crc ^ 0xFFFFFFFF) & 0xFFFFFFFF  # crc ^ ~0U


class NackError(RuntimeError):
    """Raised when the bootloader explicitly NACKs a frame."""


def send(bus: can.BusABC, can_id: int, data: bytes) -> None:
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    while True:
        try:
            bus.send(msg)
            return
        except can.CanOperationError:
            time.sleep(0.001)


def wait_ack(bus: can.BusABC) -> bool:
    """Block until ACK or NACK is received. Returns True on ACK."""
    deadline = time.monotonic() + ACK_TIMEOUT_S
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=ACK_TIMEOUT_S)
        if msg is None:
            break
        if msg.arbitration_id == CMD_ACK:
            return True
        if msg.arbitration_id == CMD_NACK:
            print("  ✗  NACK received from bootloader")
            return False
    raise TimeoutError("Timed out waiting for ACK")


def send_and_ack(bus: can.BusABC, can_id: int, data: bytes, label: str = "") -> None:
    send(bus, can_id, data)
    if not wait_ack(bus):
        raise NackError(f"NACK on: {label}" if label else "NACK")
    if label:
        print(f"  ✓  {label}")


def send_dfu_request(bus: can.BusABC, controller: Controller) -> None:
    send_and_ack(bus, CMD_REQUEST_DFU, bytes([controller]), "dfu_request")


def send_image_header(
    bus: can.BusABC,
    version: tuple[int, int, int],
    vector_addr: int,
    git_sha: bytes,  # exactly 8 bytes
    data_size: int,
    image_crc: int,
) -> None:
    print("\n[1/3] Sending image header…")

    payload1 = bytes(version) + struct.pack("<I", vector_addr)
    send_and_ack(
        bus,
        CMD_IMAGE_HEADER,
        payload1,
        f"version {version[0]}.{version[1]}.{version[2]}, vector=0x{vector_addr:08X}",
    )

    assert len(git_sha) == 8, "git_sha must be exactly 8 bytes"
    send_and_ack(bus, CMD_IMAGE_HEADER, git_sha, f"git SHA {git_sha.hex()}")

    payload3 = struct.pack("<II", data_size, image_crc)
    send_and_ack(
        bus, CMD_IMAGE_HEADER, payload3, f"data_size={data_size}, crc=0x{image_crc:08X}"
    )


def _send_block(
    bus: can.BusABC,
    block_data: bytes,
    write_addr: int,
) -> None:
    num_bytes = len(block_data)
    block_crc = crc32(block_data)

    payload_addr = struct.pack("<IH", write_addr, num_bytes - 1)
    send_and_ack(bus, CMD_FIRMWARE_DATA, payload_addr, "  addr+size")

    payload_crc = struct.pack("<I", block_crc)
    send_and_ack(bus, CMD_FIRMWARE_DATA, payload_crc, "  block CRC")

    num_chunks = (num_bytes + CHUNK_SIZE - 1) // CHUNK_SIZE
    for chunk_idx in range(num_chunks):
        chunk = block_data[chunk_idx * CHUNK_SIZE : (chunk_idx + 1) * CHUNK_SIZE]
        send(bus, CMD_FIRMWARE_DATA, chunk)

    if not wait_ack(bus):
        raise NackError(f"CRC mismatch for block at 0x{write_addr:08X}")


def send_firmware_elf(bus: can.BusABC, segments: list[tuple[int, bytes]]) -> None:
    print("\n[2/3] Sending ELF firmware segments…")

    for seg_idx, (base_addr, data) in enumerate(segments):
        print(f"\nSegment {seg_idx + 1}: addr=0x{base_addr:08X}, size={len(data)}")

        total_blocks = (len(data) + BLOCK_SIZE - 1) // BLOCK_SIZE

        for block_idx in range(total_blocks):
            offset = block_idx * BLOCK_SIZE
            block_data = data[offset : offset + BLOCK_SIZE]
            write_addr = base_addr + offset

            if write_addr >= FLASH_END:
                raise RuntimeError(f"Write out of bounds: 0x{write_addr:08X}")

            print(
                f"  Block {block_idx + 1}/{total_blocks}: "
                f"addr=0x{write_addr:08X}, size={len(block_data)}"
            )

            for attempt in range(1, MAX_BLOCK_RETRIES + 1):
                try:
                    _send_block(bus, block_data, write_addr)
                    break
                except NackError as exc:
                    if attempt < MAX_BLOCK_RETRIES:
                        print("  ↻ retrying…")
                    else:
                        raise RuntimeError(f"Segment {seg_idx + 1} failed") from exc

    print("  All segments sent.")


def send_finish(bus: can.BusABC) -> None:
    print("\n[3/3] Sending firmware-data-finish…")
    send(bus, CMD_FIRMWARE_DATA_FINISH, b"")
    print("  ✓  Done.")


def get_git_sha(repo_path: str = ".") -> bytes:
    result = subprocess.run(
        ["git", "-C", repo_path, "rev-parse", "--short=8", "HEAD"],
        capture_output=True,
        text=True,
        check=True,
    )
    sha_str = result.stdout.strip()
    return sha_str.encode("ascii")[:8].ljust(8, b"\x00")


def default_can_interface() -> str:
    return "pcan" if platform.system() == "Windows" else "socketcan"


def default_can_channel(interface: str) -> str:
    if interface == "pcan":
        return "PCAN_USBBUS1"
    return "can0"


def format_can_init_error(
    exc: can.CanInitializationError,
    interface: str,
    channel: str,
    bitrate: int,
) -> str:
    message = str(exc)
    prefix = f"Failed to open {channel} via {interface} @ {bitrate} bps"

    if interface == "pcan" and "irregularities were registered" in message:
        return (
            f"{prefix}: {message}. Check that the PCAN-USB adapter is connected, "
            "the CAN bus is powered and terminated, and the selected bitrate and "
            "channel match the hardware."
        )

    return f"{prefix}: {message}"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="CAN DFU sender for STM32 bootloader")
    p.add_argument("firmware", help="Path to flat binary firmware image (.bin)")
    p.add_argument(
        "--interface",
        default=default_can_interface(),
        choices=["pcan", "socketcan"],
        help="CAN backend (default: pcan on Windows, socketcan elsewhere)",
    )
    p.add_argument(
        "--channel",
        default=None,
        help="CAN channel (default: PCAN_USBBUS1 for pcan, can0 for socketcan)",
    )
    p.add_argument(
        "--controller",
        type=str,
        choices=["ACM", "FC", "RC", "WDAQ"],
        required=True,
        help="Controller to flash",
    )
    p.add_argument(
        "--bitrate",
        type=int,
        default=1000_000,
        help="CAN bitrate in bps (default: 1000000)",
    )
    p.add_argument(
        "--app-start",
        type=lambda x: int(x, 0),
        default=0x08020000,
        help="App flash start address (default: 0x08008000)",
    )
    p.add_argument(
        "--vector-addr",
        type=lambda x: int(x, 0),
        default=0x08020000,
        help="Vector table address (default: same as --app-start)",
    )
    p.add_argument(
        "--version",
        default="1.0.0",
        help="Firmware version major.minor.patch (default: 1.0.0)",
    )
    p.add_argument(
        "--git-sha", default=None, help="8-char git SHA (default: auto-detect from cwd)"
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    channel = args.channel or default_can_channel(args.interface)

    firmware_path = Path(args.firmware)
    if not firmware_path.exists():
        sys.exit(f"Error: firmware file not found: {firmware_path}")

    controller = CONTROLLER_STR_MAP[args.controller]

    segments = []
    if firmware_path.suffix == ".elf":
        segments = load_elf_segments(firmware_path)
    else:
        firmware = firmware_path.read_bytes()
        segments = [(args.app_start, firmware)]

    try:
        version = tuple(int(x) for x in args.version.split("."))
        assert len(version) == 3
    except Exception:
        sys.exit("Error: --version must be in 'major.minor.patch' format")

    if args.git_sha:
        git_sha = args.git_sha.encode("ascii")[:8].ljust(8, b"\x00")
    else:
        try:
            git_sha = get_git_sha()
            print(f"Git SHA (auto): {git_sha.decode()}")
        except Exception:
            git_sha = b"\x00" * 8
            print("Warning: could not read git SHA, using zeros")

    data_size = sum(len(data) for _, data in segments)
    image_crc = crc32(b"".join(data for _, data in segments))
    print(f"CRC-32: 0x{image_crc:08X}")

    print(f"\nOpening {channel} via {args.interface} @ {args.bitrate} bps…")
    try:
        with can.interface.Bus(
            interface=args.interface,
            channel=channel,
            bitrate=args.bitrate,
            can_filters=[
                {"can_id": 0x100, "can_mask": 0x7FF, "extended": False},
                {"can_id": 0x101, "can_mask": 0x7FF, "extended": False},
                {"can_id": 0x102, "can_mask": 0x7FF, "extended": False},
                {"can_id": 0x103, "can_mask": 0x7FF, "extended": False},
                {"can_id": 0x104, "can_mask": 0x7FF, "extended": False},
                {"can_id": 0x105, "can_mask": 0x7FF, "extended": False},
                {"can_id": 0x106, "can_mask": 0x7FF, "extended": False},
            ],
        ) as bus:
            try:
                send_dfu_request(bus, controller)
                send_image_header(
                    bus,
                    version=version,
                    vector_addr=args.vector_addr,
                    git_sha=git_sha,
                    data_size=data_size,
                    image_crc=image_crc,
                )
                send_firmware_elf(bus, segments)
                send_finish(bus)
                print("\n✅  DFU complete.")
            except (RuntimeError, TimeoutError) as exc:
                sys.exit(f"\n❌  DFU failed: {exc}")
    except can.CanInitializationError as exc:
        sys.exit(f"\n❌  {format_can_init_error(exc, args.interface, channel, args.bitrate)}")


if __name__ == "__main__":
    main()
