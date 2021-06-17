from pathlib import Path

from filter_generator_exception import raise_cannot_find_tag


class FactoryHeader:
    def __init__(self, project_path: Path, path: Path, filename, content: list, included_filters: list, tags: dict):
        self.project_path = project_path
        self.path = path
        self.filename = filename
        self.content = content
        self.included_filters = included_filters
        self.tags = tags

    def generate(self):
        for i in range(len(self.content)):
            if self.tags["includes-start"] in self.content[i]:
                idx = i + 1
                while self.tags["includes-end"] not in self.content[idx]:
                    self.content.pop(idx)
                    if idx >= len(self.content):
                        raise_cannot_find_tag(self.tags["includes-end"], self.filename, False)
                for j in range(len(self.included_filters)):
                    include_path = self.included_filters[j].path.parts[len(self.project_path.parts) - 1:]
                    self.content.insert(idx + j, "#include <" + "/".join(include_path) + ">\n")
                return
        raise_cannot_find_tag(self.tags["includes-start"], self.filename)

    def write(self):
        with open(self.path, 'w') as f:
            f.write("".join(self.content))


def load(project_path: Path, path: Path, filter_headers: list, tags: dict) -> FactoryHeader:
    with open(path) as f:
        content = f.readlines()
    return FactoryHeader(project_path, path, path.name, content, filter_headers, tags)
