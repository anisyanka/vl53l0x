import tkthread; tkthread.patch()
import tkinter as tk
import threading
import random
import time

root = tk.Tk()
root.geometry("500x200")
root.title("Range sensor viewer")

range_label = tk.Label(root, text=str(random.randint(0, 1000)), font=('Arial', 100))
range_label.pack(padx=20, pady=50)

def thread_run(func):
    threading.Thread(target=func).start()

@thread_run
def serial_thread():
    @tkthread.main(root)
    @tkthread.current(root)
    def serial_read_func():
        while 1:
            time.sleep(0.1)
            range_label.config(text=str(random.randint(0, 1000)))

root.mainloop()
