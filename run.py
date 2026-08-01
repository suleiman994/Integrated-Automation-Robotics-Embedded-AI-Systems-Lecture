"""Generate figures and run the integrated laboratory."""

from pathlib import Path
import subprocess
import sys


def main() -> None:
    root = Path(__file__).resolve().parent
    for command in [
        [sys.executable, str(root / "generate_figures.py")],
        [sys.executable, str(root / "experiments" / "run_integrated_lab.py")],
    ]:
        subprocess.run(command, cwd=root, check=True)


if __name__ == "__main__":
    main()
