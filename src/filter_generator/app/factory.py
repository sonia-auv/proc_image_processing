from pathlib import Path


class Factory:
    def __init__(self):
        self.enabled_filters = []


def load(path: Path, factory: Path, factory_header: Path):
    with open(Path(path, factory_header)) as f:
        factory_header_content = f.readlines()
    for i in range(len(factory_header_content)):
        if "// <FILTER_GENERATOR_HEADER_INCLUDES>" in factory_header_content[i]:
            while "// <FILTER_GENERATOR_HEADER_INCLUDES/>" not in factory_header_content[i + 1]:
                factory_header_content.pop(i + 1)
    with open(Path(path, factory_header), 'w') as f:
        f.write("".join(factory_header_content))
    with open(Path(path, factory)) as f:
        factory_content = f.readlines()
