from pathlib import Path

from factory_generator_exception import raise_cannot_find_tag


class Factory:
    def __init__(self, project_path: Path, path: Path, filename, content: list, included_item_headers: list,
                 equality_variable, create_params: list, tags: dict):
        self.project_path = project_path
        self.path = path
        self.filename = filename
        self.content = content
        self.equality_variable = equality_variable
        self.create_params = create_params
        self.included_item_headers = included_item_headers
        self.tags = tags

    def generate_instance_creation(self):
        for i in range(len(self.content)):
            if self.tags["create-start"] in self.content[i]:
                idx = i + 1
                while self.tags["create-end"] not in self.content[idx]:
                    self.content.pop(idx)
                    if idx >= len(self.content):
                        raise_cannot_find_tag(self.tags["create-end"], self.filename, False)
                for j in range(len(self.included_item_headers)):
                    class_name = self.included_item_headers[j].class_name
                    params = ", ".join(self.create_params)
                    if j == 0:
                        line = '\tif(' + self.equality_variable + ' == "' + class_name + '"){\n\t\treturn std::make_unique<' + class_name + '>(' + params + ');\n\t}\n'
                    else:
                        line = '\telse if(' + self.equality_variable + ' == "' + class_name + '"){\n\t\treturn std::make_unique<' + class_name + '>(' + params + ');\n\t}\n'
                    self.content.insert(idx + j, line)
                return
        raise_cannot_find_tag(self.tags["create-start"], self.filename)

    def generate_item_headers_list(self):
        for i in range(len(self.content)):
            if self.tags["list-start"] in self.content[i]:
                idx = i + 1
                while self.tags["list-end"] not in self.content[idx]:
                    self.content.pop(idx)
                    if idx >= len(self.content):
                        raise_cannot_find_tag(self.tags["list-end"], self.filename, False)
                self.content.insert(idx,
                                    '\treturn ' + ';"\n\t\t'.join(
                                        ['"' + a.class_name for a in self.included_item_headers]) + '";\n')
                return
        raise_cannot_find_tag(self.tags["list-start"], self.filename)

    def generate(self):
        self.generate_instance_creation()
        self.generate_item_headers_list()

    def write(self):
        with open(self.path, 'w') as f:
            f.write("".join(self.content))


def load(project_path: Path, path: Path, item_headers: list, equality_variable, create_params: list,
         tags: dict) -> Factory:
    with open(path) as f:
        content = f.readlines()
    return Factory(project_path, path, path.name, content, item_headers, equality_variable, create_params, tags)
