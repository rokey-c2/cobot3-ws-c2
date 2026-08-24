class BoxCounter:
    def __init__(self, capacity: int = 10):
        self.capacity = capacity
        self.count = 0

    @property
    def is_full(self) -> bool:
        return self.count >= self.capacity

    def add(self):
        self.count += 1

    def reset(self):
        self.count = 0
