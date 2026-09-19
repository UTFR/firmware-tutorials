import argparse
import os
import subprocess
from scripts.scripts import write_file_and_format


ROOT_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir)
DEFAULT_GENERATED_DIR = os.path.join(ROOT_DIR, "generated/git")

H_CONTENT = """\
#ifndef GIT_VERSION_H
#define GIT_VERSION_H

#define GIT_HASH "{hash}"
#define BRANCH_NAME "{branch}" // max length of "16"

#endif
"""


def get_hash() -> str:
    result = subprocess.run(
        ["git", "rev-parse", "--short", "HEAD"],
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout.strip()


def get_branch() -> str:
    result = subprocess.run(
        ["git", "rev-parse", "--abbrev-ref", "HEAD"],
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout.strip()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate CAN types from DBC.")
    parser.add_argument(
        "--output-dir",
        "-o",
        default=DEFAULT_GENERATED_DIR,
        help="directory to place generated files",
    )
    args = parser.parse_args()

    hash = get_hash()
    branch = get_branch()
    content = H_CONTENT.format(hash=hash, branch=branch)
    write_file_and_format(f"{args.output_dir}/git.h", content)
