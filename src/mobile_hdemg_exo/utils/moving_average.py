class MovingAverage:
    def __init__(self, window_size=100):
        self.window_size = window_size
        self.data = []

    def moving_avg(self, value):
        self.data.append(value)
        if len(self.data) > self.window_size:
            self.data.pop(0)
        return sum(self.data) / len(self.data)
