from pathlib import Path


class Filter:
    def __init__(self, path: Path, filename, content: list):
        self.path = path
        self.filename = filename
        self.content = content


def load(path: Path) -> Filter:
    with open(path) as f:
        content = f.readlines()
    return Filter(path, path.name, content)


def load_all(paths: list) -> list:
    filters = list()
    for path in paths:
        filters.append(load(path))
    return filters
