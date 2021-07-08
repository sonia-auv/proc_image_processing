import logging
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.joinpath('app').absolute()))

from app.factory_generator import FactoryGenerator
from app.tools import validate_and_get_conf

if __name__ == "__main__":
    try:
        conf = validate_and_get_conf()
        for factory in conf["factories"]:
            print("Generating factory " + factory["name"] + "...", end=" ")
            generator = FactoryGenerator(conf["project-path"], factory, conf["tags"])
            generator.generate()
            print("Done!")
    except:
        logging.warning("Cannot generate, please fix errors first!")
