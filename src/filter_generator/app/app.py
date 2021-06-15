from pathlib import Path

import factory as fa
import factory_header as fah
import filter as f
from tools import get_conf, get_files_from_path

if __name__ == '__main__':
    conf = get_conf()
    filters = f.load_all(
        get_files_from_path(conf["filters-path"]),
        conf["excluded-filters"]
    )
    factory_header = fah.load(
        conf["project-path"],
        Path(conf["factory-path"], conf["factory-header-filename"]),
        filters,
        conf["tags"]["factory-header"]
    )
    factory_header.generate()
    factory = fa.load(
        conf["project-path"],
        Path(conf["factory-path"], conf["factory-filename"]),
        filters,
        conf["tags"]["factory"]
    )
