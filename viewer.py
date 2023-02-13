import tkthread; tkthread.patch()
import tkinter as tk
import threading
import random
import time
import serial

SERIAL_DEV_NAME = "/dev/tty.usbserial-120"

root = tk.Tk()
root.geometry("500x200")
root.title("Range sensor viewer")

range_label = tk.Label(root, text=str(random.randint(0, 1000)), font=('Arial', 100))
range_label.pack(pady=50)

sensor = serial.Serial(SERIAL_DEV_NAME, baudrate=115200, timeout = 1)

def thread_run(func):
    threading.Thread(target=func).start()

@thread_run
def serial_thread():
    @tkthread.main(root)
    @tkthread.current(root)
    def serial_read_func():
        while 1:
            text_int = int(sensor.readline())
            if text_int >= 8189:
                text = "Too far"
            else:
                text = str(text_int/10) + " cm"

            range_label.config(text=text)

root.mainloop()
