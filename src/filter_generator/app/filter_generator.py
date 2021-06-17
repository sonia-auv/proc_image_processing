from pathlib import Path

import factory as fa
import factory_header as fah
import filter_header as f
from tools import validate_and_get_conf, get_files_from_path


class FilterGenerator:
    def __init__(self):
        self.conf = validate_and_get_conf()
        self.filter_headers = f.load_all(
            get_files_from_path(self.conf["filters-path"], True, ".h"),
            self.conf["excluded-filter-headers"],
            self.conf["tags"]["filter-headers"]
        )
        self.factory_header = fah.load(
            self.conf["project-path"],
            Path(self.conf["factory-path"], self.conf["factory-header-filename"]),
            self.filter_headers,
            self.conf["tags"]["factory-header"]
        )
        self.factory = fa.load(
            self.conf["project-path"],
            Path(self.conf["factory-path"], self.conf["factory-filename"]),
            self.filter_headers,
            self.conf["tags"]["factory"]
        )

    def generate(self):
        self.factory_header.generate()
        self.factory.generate()
        self.factory_header.write()
        self.factory.write()
