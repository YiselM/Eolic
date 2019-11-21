import continuous_threading
import time

class CountingThread(continuous_threading.PausableThread):
    def __init__(self):
        global counter
        super().__init__()
        counter = 0

    def _run(self):
        global counter
        counter += 1

global counter
th = CountingThread()
th.start()
print(str(counter))
time.sleep(2)

th.stop()
print(str(counter))
time.sleep(2)
