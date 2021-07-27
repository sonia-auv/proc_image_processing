import logging


class FactoryGeneratorException(Exception):
    def __init__(self, msg):
        logging.error(msg)
        self.msg = msg


def raise_cannot_find_tag(tag, filename, start=True, surrounded=True):
    s = "'" + tag + "' in '" + filename + "'."
    if surrounded:
        if start:
            raise FactoryGeneratorException("Cannot find starting tag " + s)
        raise FactoryGeneratorException("Cannot find ending tag " + s)
    raise FactoryGeneratorException("Cannot find tag " + s)
