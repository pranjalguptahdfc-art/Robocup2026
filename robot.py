import math, time
import ev3_dc as ev3
import config

class Robot:
    def __init__(self, no_ev3=False):
        self.no_ev3 = no_ev3
        if no_ev3:
            self.w1 = self.w2 = self.w3 = self.dribbler = None
            return
        r = ev3.EV3(protocol=ev3.BLUETOOTH, host=config.EV3_ADDRESS)
        self.w1 = ev3.Motor(ev3.PORT_B, ev3_obj=r)
        self.w2 = ev3.Motor(ev3.PORT_C, ev3_obj=r)
        self.w3 = ev3.Motor(ev3.PORT_D, ev3_obj=r)
        self.dribbler = ev3.Motor(ev3.PORT_A, ev3_obj=r)

    def omni(self, vx, vy, omega):
        if self.no_ev3:
            print(f"omni vx={vx:>3} vy={vy:>3} om={omega:>3}")
            return
        s1 = vx + omega
        s2 = -0.5*vx + (math.sqrt(3)/2)*vy + omega
        s3 = -0.5*vx - (math.sqrt(3)/2)*vy + omega
        m = max(abs(s1), abs(s2), abs(s3), 100)
        f = 100.0 / m
        self.w1.start(int(s1*f)); self.w2.start(int(s2*f)); self.w3.start(int(s3*f))

    def stop(self):
        if self.no_ev3:
            return
        for w in (self.w1, self.w2, self.w3):
            w.stop(brake=True)

    def dribbler_forward(self, speed=None):
        if self.no_ev3:
            return
        s = config.DIBBLE_SPEED if speed is None else speed
        self.dribbler.start(speed=s)

    def dribbler_reverse(self, speed=None):
        if self.no_ev3:
            return
        s = config.SHOOT_SPEED if speed is None else speed
        self.dribbler.start(speed=-s)

    def shoot(self):
        if self.no_ev3:
            print(">>> SHOOT <<<")
            time.sleep(config.SHOOT_MS / 1000.0)
            return
        self.dribbler_reverse()
        time.sleep(config.SHOOT_MS / 1000.0)
        self.dribbler.stop(brake=False)

    def idle(self):
        self.stop()
        if not self.no_ev3:
            self.dribbler.stop(brake=True)
