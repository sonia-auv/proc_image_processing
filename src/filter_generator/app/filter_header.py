from pathlib import Path

from filter_generator_exception import raise_cannot_find_tag


class FilterHeader:
    def __init__(self, path: Path, filename, content: list, class_name):
        self.path = path
        self.filename = filename
        self.content = content
        self.class_name = class_name


def load(path: Path, tags: dict) -> FilterHeader:
    with open(path) as f:
        content = f.readlines()
    for i in range(len(content)):
        if tags["class-name"] in content[i]:
            class_name = content[i].strip("\n\t").split(tags["class-name-separator"])
            if len(class_name) != 2:
                raise_cannot_find_tag(tags["class-name"] + tags["class-name-separator"], path.name, surrounded=False)
            return FilterHeader(path, path.name, content, class_name[1])
    raise_cannot_find_tag(tags["class-name"], path.name, surrounded=False)


def load_all(paths: list, excluded_filter_headers: list, tags: dict) -> list:
    filter_headers = list()
    for path in paths:
        if path.name not in excluded_filter_headers:
            try:
                filter_headers.append(load(path, tags))
            except:
                pass
    return filter_headers
