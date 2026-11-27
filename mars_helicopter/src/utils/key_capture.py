import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

import threading
import time

class KeyCapture:
    def __init__(self):
        self.pressed_keys = set()  # Use a set to store unique key presses
        self.stop_thread = False

    def getKey(self):
        if os.name == 'nt':  # Windows
            while not self.stop_thread:
                if msvcrt.kbhit():
                    key = msvcrt.getch().decode() if sys.version_info[0] >= 3 else msvcrt.getch()
                    if key == '\r':  # Handle Enter
                        key = '\n'
                    self.pressed_keys.add(key)
                time.sleep(0.01)  # Slight delay to avoid hogging CPU
        else:  # Unix-like
            settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            try:
                while not self.stop_thread:
                    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                    if rlist:
                        key = sys.stdin.read(1)
                        self.pressed_keys.add(key)
            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def start_listening(self):
        self.thread = threading.Thread(target=self.getKey)
        self.thread.start()

    def stop_listening(self):
        self.stop_thread = True
        self.thread.join()

    def get_pressed_keys(self):
        keys = list(self.pressed_keys)  # Make a copy to safely access
        self.pressed_keys.clear()  # Clear the set after accessing
        return keys