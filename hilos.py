# EXAMPLE 2
import threading
import time

global sw = 0


class myThread(threading.Thread):
    def __init__(self, threadId, name, count):
        threading.Thread.__init__(self)
        self.threadId = threadId
        self.name = name
        self.count = count

    def run(self):
        print("Starting: " + self.name + "\n")
        print_time(self.name, 1,self.count)
        print("Exiting: " + self.name + "\n")

class myThread2(threading.Thread):
    def __init__(self, threadId, name, count):
        threading.Thread.__init__(self)
        self.threadId = threadId
        self.name = name
        self.count = count

    def run(self):
        print("Starting: " + self.name + "\n")
        print_time(self.name, 1,self.count)
        print("Exiting: " + self.name + "\n")


def print_time(name, delay, count):
    while sw == 0:
        time.sleep(delay)
        print ("%s: %s %s" % (name, time.ctime(time.time()), count) + "\n")
        count -= 1
        
        if


threadLock = threading.Lock()

thread1 = myThread(1, "Payment", 5)
thread2 = myThread2(2, "Sending Email", 10)
thread3 = myThread2(3, "Loading Page", 3)

threadLock.acquire()
thread1.start()
thread2.start()
thread1.join()
thread2.join()
threadLock.release()

thread3.start()
thread3.join()

print("Done main thread")
