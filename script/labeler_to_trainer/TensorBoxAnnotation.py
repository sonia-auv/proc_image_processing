import json

# TODO:
# -----

# Class
#    - Read and store (runtime Python obj) json file data
#    - Write json file from class

class RectArea:
    def __init__(self, name, x1, x2, y1, y2):
        self.name = name
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

    def toDict(self):
        dict = {}
        for rect in self.rects:
            dict[rect.name] = rect.toDict()
        return dict


if __name__ == "__main__":
    rects = [RectArea('rec1', 5, 10, 15, 20), RectArea('rec2', 25, 30, 35, 40)]
    image = TrainingImage("456", rects)
    image2 = TrainingImage("123", rects)
    dict_tmp = {}
    dict_tmp["123"] = image.toDict()
    dict_tmp["456"] = image.toDict()
    print json.dumps(dict_tmp, sort_keys=True, indent=4, separators=(',', ': '))