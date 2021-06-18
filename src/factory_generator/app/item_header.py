from pathlib import Path

from factory_generator_exception import raise_cannot_find_tag


class ItemHeader:
    def __init__(self, path: Path, filename, content: list, class_name):
        self.path = path
        self.filename = filename
        self.content = content
        self.class_name = class_name


def load(path: Path, tags: dict) -> ItemHeader:
    with open(path) as f:
        content = f.readlines()
    for i in range(len(content)):
        if tags["class-name"] in content[i]:
            class_name = content[i].strip("\n\t").split(tags["class-name-separator"])
            if len(class_name) != 2:
                raise_cannot_find_tag(tags["class-name"] + tags["class-name-separator"], path.name, surrounded=False)
            return ItemHeader(path, path.name, content, class_name[1])
    raise_cannot_find_tag(tags["class-name"], path.name, surrounded=False)


def load_all(paths: list, excluded_item_headers: list, tags: dict) -> list:
    item_headers = list()
    for path in paths:
        if path.name not in excluded_item_headers:
            try:
                item_headers.append(load(path, tags))
            except:
                pass
    return item_headers
