from pathlib import Path


class Factory:
    def __init__(self, project_path: Path, path: Path, filename, content: list, included_filters: list, tags: dict):
        self.project_path = project_path
        self.path = path
        self.filename = filename
        self.content = content
        self.included_filters = included_filters
        self.tags = tags

    def generate(self):
        for i in range(len(self.content)):
            if self.tags["filters-list-start"] in self.content[i]:
                idx = i + 1
                while self.tags["filters-list-end"] not in self.content[idx]:
                    self.content.pop(idx)
                for j in range(len(self.included_filters)):
                    # TODO
                    pass
            if self.tags["instance-creation-start"] in self.content[i]:
                idx = i + 1
                while self.tags["instance-creation-end"] not in self.content[idx]:
                    self.content.pop(idx)
                for j in range(len(self.included_filters)):
                    # TODO
                    pass
        # with open(self.path, 'w') as f:
        #     f.write("".join(self.content))


def load(project_path, path: Path, filters: list, tags: dict) -> Factory:
    with open(path) as f:
        content = f.readlines()
    return Factory(project_path, path, path.name, content, filters, tags)
