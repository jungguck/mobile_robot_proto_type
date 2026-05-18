import os
import subprocess


class ProcessLauncher:
    def __init__(self, command, name):
        self.command = command
        self.name = name
        self.process = None

    def start(self):
        if self.process is not None and self.process.poll() is None:
            return False

        env = os.environ.copy()
        self.process = subprocess.Popen(
            self.command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=env,
        )
        return True

    def stop(self):
        if self.process is None or self.process.poll() is not None:
            return False

        self.process.terminate()
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.process = None
        return True
