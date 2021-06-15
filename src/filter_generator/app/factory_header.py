from pathlib import Path


class FactoryHeader:
    def __init__(self, path: Path, filename, content: list):
        self.path = path
        self.filename = filename
        self.content = content
        self.included_filters = []

    def generate(self):
        for i in range(len(self.content)):
            if "// <FILTER_GENERATOR_HEADER_INCLUDES>" in self.content[i]:
                while "// <FILTER_GENERATOR_HEADER_INCLUDES/>" not in self.content[i + 1]:
                    self.content.pop(i + 1)
        with open(self.path, 'w') as f:
            f.write("".join(self.content))


def load(path: Path) -> FactoryHeader:
    with open(path) as f:
        content = f.readlines()
    return FactoryHeader(path, path.name, content)
