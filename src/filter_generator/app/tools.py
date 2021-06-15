import json
import logging
from pathlib import Path

import yaml
from jsonschema import validate, ValidationError

current_path = Path(__file__).parent.absolute()


def get_conf() -> dict:
    schema_path = current_path.joinpath('../conf-schema.json')
    conf_path = current_path.joinpath('../conf.yml')
    if schema_path.is_file():
        with open(schema_path) as f:
            schema = json.load(f)
        with open(conf_path) as f:
            conf = yaml.load(f, Loader=yaml.FullLoader)
        try:
            validate(conf, schema)
            validate_and_fix_path(conf, "project-path")
            conf["filters-path"] = conf["project-path"].joinpath(conf["filters-path"])
            conf["factory-path"] = conf["project-path"].joinpath(conf["factory-path"])
            return conf
        except ValidationError as ve:
            logging.warning("Invalid configuration file, default values will be used. Error: " + str(ve))
            return get_default_conf_values()
    return get_default_conf_values()


def validate_and_fix_path(conf: dict, key: str):
    path = Path(conf[key])
    if not path.is_absolute() and not path.exists():
        path = Path(current_path, '../', conf[key])
        if not path.exists():
            raise ValidationError("Invalid path: " + str(path))
        conf[key] = path
    else:
        conf[key] = Path(conf[key])


def get_default_conf_values() -> dict:
    project_path = current_path.joinpath('../../proc_image_processing').absolute()
    return {
        "project-root": project_path,
        "filters-path": project_path.joinpath('filters/'),
        "factory-path": project_path.joinpath('server/'),
        "factory-filename": "filter_factory.cc",
        "factory-header-filename": "filter_factory.h"
    }


def get_files_from_path(path: Path, recurse=False) -> list:
    if recurse:
        paths = path.glob('*/**')
    else:
        paths = path.glob('*')
    out = []
    for file in [i for i in paths if i.is_file()]:
        out.append(file)
    return out
