import json

# TODO:
# -----

# Class
#    - Read and store (runtime Python obj) json file data
#    - Write json file from class

class RectArea:
    def __init__(self, x1, x2, y1, y2):
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2

    def toDict(self):
        return {'x1': self.x1, 'x2': self.x2, 'y1': self.y1, 'y2': self.y2}

class TrainingImage:
    def __init__(self, image_path, rects):
        self.path = image_path
        self.rects = rects

    def toJson(self):
        return json.dumps({'image_path': self.path, 'rects': [_.toDict() for _ in self.rects ] },
                            sort_keys=True, indent=4, separators=(',', ': '))


if __name__ == "__main__":
    rects = [ RectArea(5, 10, 15, 20), RectArea(25, 30, 35, 40)]
    image = TrainingImage("/usr/local/", rects)

    image.toJson()