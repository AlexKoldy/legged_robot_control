class FiniteStateMachine:
    def __init__(self, swing_duration: float):
        """
        TODO
        """
        self.support_modes = {
            0: "DOUBLE",
            1: "LEFT",
            2: "RIGHT",
        }
        self.state = 1
        self.swing_duration = swing_duration

    def get_state(self, t: float) -> str:
        """
        TODO
        """
        if t % (2 * self.swing_duration) > self.swing_duration:
            return "RIGHT"
        else:
            return "LEFT"


if __name__ == "__main__":
    import time

    fsm = FiniteStateMachine(3)
    time_before = time.time()
    while True:
        t = time.time() - time_before
        state = fsm.get_state(t)
        print(f"{t}: {state}")
