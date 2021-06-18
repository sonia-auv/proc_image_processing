import json
from pathlib import Path

import yaml
from jsonschema import validate

from factory_generator_exception import FactoryGeneratorException

current_path = Path(__file__).parent.absolute()


def get_conf_schema() -> dict:
    schema_path = current_path.joinpath('../conf-schema.json')
    if schema_path.is_file():
        with open(schema_path) as f:
            schema = json.load(f)
        return schema
    raise FactoryGeneratorException("Cannot find configuration schema file (conf-schema.json)!")


def get_conf() -> dict:
    conf_path = current_path.joinpath('../conf.yml')
    if conf_path.is_file():
        with open(conf_path) as f:
            conf = yaml.load(f, Loader=yaml.FullLoader)
        return conf
    raise FactoryGeneratorException("Cannot find configuration file (conf.yml)!")


def validate_and_fix_path(p):
    path = Path(p)
    if not path.is_absolute() and not path.exists():
        path = Path(current_path, '../', p)
        if not path.exists():
            raise FactoryGeneratorException("Cannot find path '" + p + "'.")
    return path


def get_files_from_path(path: Path, recurse=False, extension="") -> list:
    if recurse:
        paths = path.rglob('*' + extension)
    else:
        paths = path.glob('*' + extension)
    out = []
    for file in [i for i in paths if i.is_file()]:
        out.append(file)
    return out


def validate_and_get_conf() -> dict:
    schema = get_conf_schema()
    conf = get_conf()
    validate(conf, schema)
    conf["project-path"] = validate_and_fix_path(conf["project-path"])
    for factory in conf["factories"]:
        factory["path"] = conf["project-path"].joinpath(factory["path"])
        factory["items-path"] = conf["project-path"].joinpath(factory["items-path"])
    return conf
