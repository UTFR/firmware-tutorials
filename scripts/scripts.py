import os
import subprocess
from pathlib import Path


def write_file_and_format(path: Path, content: str):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")

    p = subprocess.run(
        ["clang-format"],
        input=content,
        text=True,
        capture_output=True,
        check=True,
    )
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(p.stdout)

    os.replace(tmp, path)
