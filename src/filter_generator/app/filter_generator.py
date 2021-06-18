from pathlib import Path

import factory as fa
import factory_header as fah
import filter_header as f
from tools import get_files_from_path


class FilterGenerator:
    def __init__(self, project_path: Path, conf: dict, tags: dict):
        self.project_path = project_path
        self.conf = conf
        self.tags = tags
        self.filter_headers = f.load_all(
            get_files_from_path(self.conf["items-path"], True, ".h"),
            self.conf["excluded-items"],
            self.tags["items-headers"]
        )
        self.factory_header = fah.load(
            self.project_path,
            Path(self.conf["path"], self.conf["header-filename"]),
            self.filter_headers,
            self.tags["factory-header"]
        )
        self.factory = fa.load(
            self.project_path,
            Path(self.conf["path"], self.conf["filename"]),
            self.filter_headers,
            self.tags["factory"]
        )

    def generate(self):
        self.factory_header.generate()
        self.factory.generate()
        self.factory_header.write()
        self.factory.write()
