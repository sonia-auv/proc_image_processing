from pathlib import Path


class Factory:
    def __init__(self, path: Path, filename, content: list, included_filters: list):
        self.path = path
        self.filename = filename
        self.content = content
        self.included_filters = []

    def generate(self):
        pass


def load(path: Path) -> Factory:
    with open(path) as f:
        content = f.readlines()
    return Factory(path, path.name, content)
