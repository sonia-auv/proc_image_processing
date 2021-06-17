from pathlib import Path

from filter_generator_exception import raise_cannot_find_tag


class Factory:
    def __init__(self, project_path: Path, path: Path, filename, content: list, included_filters: list, tags: dict):
        self.project_path = project_path
        self.path = path
        self.filename = filename
        self.content = content
        self.included_filters = included_filters
        self.tags = tags

    def generate_instance_creation(self):
        for i in range(len(self.content)):
            if self.tags["instance-creation-start"] in self.content[i]:
                idx = i + 1
                while self.tags["instance-creation-end"] not in self.content[idx]:
                    self.content.pop(idx)
                    if idx >= len(self.content):
                        raise_cannot_find_tag(self.tags["instance-creation-end"], self.filename, False)
                for j in range(len(self.included_filters)):
                    class_name = self.included_filters[j].class_name
                    self.content.insert(idx + j,
                                        "\tcase '" +
                                        class_name +
                                        "':\n\t\treturn new " +
                                        class_name +
                                        "(globalParams);\n"
                                        )
                return
        raise_cannot_find_tag(self.tags["instance-creation-start"], self.filename)

    def generate_filters_list(self):
        for i in range(len(self.content)):
            if self.tags["filters-list-start"] in self.content[i]:
                idx = i + 1
                while self.tags["filters-list-end"] not in self.content[idx]:
                    self.content.pop(idx)
                    if idx >= len(self.content):
                        raise_cannot_find_tag(self.tags["filters-list-end"], self.filename, False)
                self.content.insert(idx,
                                    "\treturn '" + ";".join([a.class_name for a in self.included_filters]) + "';\n")
                return
        raise_cannot_find_tag(self.tags["filters-list-start"], self.filename)

    def generate(self):
        self.generate_instance_creation()
        self.generate_filters_list()

    def write(self):
        with open(self.path, 'w') as f:
            f.write("".join(self.content))


def load(project_path, path: Path, filters: list, tags: dict) -> Factory:
    with open(path) as f:
        content = f.readlines()
    return Factory(project_path, path, path.name, content, filters, tags)
