import logging

from app.filter_generator import FilterGenerator
from tools import validate_and_get_conf

if __name__ == "__main__":
    try:
        conf = validate_and_get_conf()
        for factory in conf["factories"]:
            logging.info("Generating factory " + factory["name"] + "!")
            generator = FilterGenerator(conf["project-path"], factory, conf["tags"])
            generator.generate()
    except:
        logging.warning("Cannot generate, please fix errors first!")
