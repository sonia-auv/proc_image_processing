import logging

from app.filter_generator import FilterGenerator

if __name__ == "__main__":
    try:
        generator = FilterGenerator()
        generator.generate()
    except:
        logging.warning("Cannot generate, please fix errors first!")
