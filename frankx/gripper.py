import time
from threading import Thread
from multiprocessing import Process

from _frankx import Gripper as _Gripper
from _frankx import CommandException


def loop(x):
    t = time.time()
    i = 0
    while 1:
        if time.time() - t > 1:
            print(i)
            i+=1
            t = time.time()


class Gripper(_Gripper):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.gripper_process = None

    def move_async(self, width) -> Thread:
        p = Thread(target=self.move_unsafe, args=(width, ), daemon=True)
        p.start()
        return p

    def move_async_grasp(self, width) -> Thread:
        p = Thread(target=self.clamp, daemon=True)
        p.start()
        return p


    def move_unsafe_async(self, width):

        if self.gripper_process is not None and self.gripper_process.is_alive():
            self.gripper_process.terminate()
            try:
                self.stop()
            except CommandException:
                pass
        self.gripper_process = Process(target=self.move_unsafe, args=(width,), daemon=True)
        # p = MoveThread(loop, (width, ))
        self.gripper_process.start()
