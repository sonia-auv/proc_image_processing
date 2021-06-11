class Model:
    def __init__(self):
        self._data = {}

    def get(self, key):
        return self._data[key]

    def put(self, key, value):
        self._data[key] = value

    def exists(self, key):
        return key in self._data
