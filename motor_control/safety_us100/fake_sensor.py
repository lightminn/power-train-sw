class FakeUs100:
    def __init__(self, readings):
        self._readings = list(readings)
        self._index = 0

    def read(self):
        if not self._readings:
            return None
        if self._index < len(self._readings):
            value = self._readings[self._index]
            self._index += 1
            return value
        return self._readings[-1]
