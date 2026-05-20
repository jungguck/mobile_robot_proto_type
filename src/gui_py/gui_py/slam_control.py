import os
import subprocess
import signal


class ProcessLauncher:
    def __init__(self, command, name):
        self.command = command
        self.name = name
        self.process = None

    def start(self):
        if self.process is not None and self.process.poll() is None:
            return False, f"{self.name} is already running"

        try:
            env = os.environ.copy()
            # Use start_new_session to ensure we can kill the whole process group if needed
            self.process = subprocess.Popen(
                self.command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=env,
                preexec_fn=os.setsid
            )
            return True, f"{self.name} started (PID: {self.process.pid})"
        except Exception as e:
            return False, f"Failed to start {self.name}: {str(e)}"

    def stop(self):
        if self.process is None or self.process.poll() is not None:
            return False, f"{self.name} is not running"

        try:
            # Kill the process group
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            self.process.wait()
        except Exception as e:
            return False, f"Error stopping {self.name}: {str(e)}"

        self.process = None
        return True, f"{self.name} stopped"

    def is_running(self):
        return self.process is not None and self.process.poll() is None
