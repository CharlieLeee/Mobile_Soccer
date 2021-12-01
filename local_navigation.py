 class obstacle_avoidance() : 
    
    def __init__(self):
        speed0 = 100       # nominal speed 
        speedGain = 2      # gain used with ground gradient

        obstThrL = 10      # low obstacle threshold to switch state 1->0
        obstThrH = 20      # high obstacle threshold to switch state 0->1
        obstSpeedGain = 5  # /100 (actual gain: 5/100=0.05)

        state = 1          # 0=gradient, 1=obstacle avoidance
        obst = [0,0]       # measurements from left and right prox sensors

    def motors(self, left, right):
        return {
            "motor.left.target": [left],
            "motor.right.target": [right],
        }

    def obstacle_update(self, node, variables):
        try:
            prox_horizontal = variables["prox.horizontal"]
            obst = [prox_horizontal[0], prox_horizontal[4]]
            speed_left = self.speed0 + self.obstSpeedGain * (obst[0] // 100)
            speed_right = self.speed0 + self.obstSpeedGain * (obst[1] // 100) 
            node.send_set_variables(self.motors(speed_left, speed_right))
        except KeyError:
            pass  # prox.horizontal not found