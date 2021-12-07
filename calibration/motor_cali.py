import sys
sys.path.append("..")
print(sys.path)
from Thymio import Thymio
import time

PORT = "COM6"

def transitional_vel(speed = 100, span = 5):
    th.set_var("motor.left.target", speed)
    th.set_var("motor.right.target", speed)
    starter = time.time()
    while time.time() - starter < span:
        pass
    stop()
    print(F"Speed: {speed}, Timespan: {span}")

def rotational_vel(dir="left", speed = 100, omega = 50, span = 5):
    ls = speed - omega
    rs = speed + omega
    ls = ls if ls > 0 else 2 ** 16 + ls
    rs = rs if rs > 0 else 2 ** 16 + rs
    if dir == "left":
        th.set_var("motor.left.target", ls)
        th.set_var("motor.right.target", rs)
    else:
        th.set_var("motor.left.target", rs)
        th.set_var("motor.right.target", ls)
    starter = time.time()
    while time.time() - starter < span:
        pass
    stop()
    print(F"Speed: {speed}, Timespan: {span}")

def stop():    
    th.set_var("motor.left.target", 0)
    th.set_var("motor.right.target", 0)

with Thymio.serial(port=PORT, refreshing_rate=0.1) as th:
    while True:
        try:
            if th["button.forward"]:
                transitional_vel(speed=100, span=10)
            elif th["button.left"]:
                rotational_vel("left",speed=0, omega=91, span=5)
            elif th["button.right"]:
                rotational_vel("right",speed=0, omega=-91, span=5)
            elif th["button.center"]:
                stop()
                th.close()
                break
        except KeyError:
            pass
        time.sleep(0.2)