import os
import signal
from subprocess import Popen
from pathlib import Path

def launch(cmd, cwd=None):
    """Launch a command."""
    print("Running command: ", " ".join(map(str, cmd)))
    process = Popen(
        args=cmd,
        encoding="utf-8",
        cwd=cwd,
    )
    try:
        process.wait()
    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        if process.poll() is None:
            try:
                print("do killpg")
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except:
                print("killpg failed")
        if process.returncode != 0:
            print(f"return code {process.returncode}")
            exit(1)


def is_valid_file(arg):
    """Argparse validator for files to check for their existence."""
    if not os.path.exists(arg):
        raise FileNotFoundError("The file %s does not exist!" % arg)
    return Path(arg)


def does_not_exists(arg):
    """Argparse validator for files to check they do not exist."""
    if os.path.exists(arg):
        raise FileExistsError("The file %s already exists!" % arg)
    return Path(arg)