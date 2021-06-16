from pathlib import Path


class Filter:
    def __init__(self, path: Path, filename, content: list, class_name):
        self.path = path
        self.filename = filename
        self.content = content
        self.class_name = class_name


def load(path: Path, tags: dict) -> Filter:
    with open(path) as f:
        content = f.readlines()
    class_name = ""
    for i in range(len(content)):
        if tags["class-name"] in content[i]:
            class_name = content[i].strip("\n\t").split(tags["class-name-separator"])[1]
    return Filter(path, path.name, content, class_name)


def load_all(paths: list, excluded_filters: list, tags: dict) -> list:
    filters = list()
    for path in paths:
        if path.name not in excluded_filters:
            filters.append(load(path, tags))
    return filters
