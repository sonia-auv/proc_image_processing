from pathlib import Path

import factory as fa
import factory_header as fah
import item_header as f
from tools import get_files_from_path


class FactoryGenerator:
    def __init__(self, project_path: Path, conf: dict, tags: dict):
        self.project_path = project_path
        self.conf = conf
        self.tags = tags
        self.item_headers = self.load_items()
        self.factory_header = self.load_factory_header()
        self.factory = self.load_factory()

    def load_items(self):
        return f.load_all(
            get_files_from_path(self.conf["items-path"], self.conf["recurse"], ".h"),
            self.conf["excluded-items"],
            self.tags["item-headers"]
        )

    def load_factory(self):
        return fa.load(
            self.project_path,
            Path(self.conf["path"], self.conf["filename"]),
            self.item_headers,
            self.conf["equality-variable"],
            self.conf["create-params"],
            self.tags["factory"]
        )

    def load_factory_header(self):
        return fah.load(
            self.project_path,
            Path(self.conf["path"], self.conf["header-filename"]),
            self.item_headers,
            self.tags["factory-header"]
        )

    def generate(self):
        self.factory_header.generate()
        self.factory.generate()
        self.factory_header.write()
        self.factory.write()
