class Model:
    def __init__(self):
        self._data = {}

    def get(self, key):
        return self._data[key]

    def put(self, key, value):
        self._data[key] = value
        return self

    def exists(self, key) -> bool:
        return key in self._data
