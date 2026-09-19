#!/usr/bin/env python3
import os
import shutil
import subprocess
import sys


def main():
    gcc = shutil.which("arm-none-eabi-gcc")
    if gcc is None:
        print("arm-none-eabi-gcc not found in PATH", file=sys.stderr)
        return 1

    # Query sysroot (Newlib location)
    try:
        sysroot = subprocess.check_output([gcc, "-print-sysroot"], text=True).strip()
    except subprocess.CalledProcessError:
        print("Failed to query GCC sysroot", file=sys.stderr)
        return 1

    clang_tidy = shutil.which("clang-tidy")
    if clang_tidy is None:
        print("clang-tidy not found in PATH", file=sys.stderr)
        return 1

    cmd = [
        clang_tidy,
        *sys.argv[1:],  # pass pre-commit args
        f"--extra-arg=--target=arm-none-eabi",
        f"--extra-arg=--sysroot={sysroot}",
    ]

    return subprocess.call(cmd)


if __name__ == "__main__":
    sys.exit(main())
