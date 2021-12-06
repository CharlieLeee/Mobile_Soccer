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
    if dir == "left":
        th.set_var("motor.left.target", speed - omega)
        th.set_var("motor.right.target", speed + omega)
    else:
        th.set_var("motor.left.target", speed + omega)
        th.set_var("motor.right.target", speed - omega)
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
                transitional_vel(speed=500, span=2)
            elif th["button.left"]:
                rotational_vel("left",speed=100, omega=20, span=5)
            elif th["button.right"]:
                rotational_vel("right",speed=100, omega=20, span=5)
            elif th["button.center"]:
                stop()
                th.close()
                break
        except KeyError:
            pass
        time.sleep(0.2)