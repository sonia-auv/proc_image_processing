import logging


class FilterGeneratorException(Exception):
    def __init__(self, msg):
        logging.error(msg)
        self.msg = msg


def raise_cannot_find_tag(tag, filename, start=True):
    s = "(" + tag + ") in " + filename + "."
    if start:
        raise FilterGeneratorException("Cannot find starting tag " + s)
    raise FilterGeneratorException("Cannot find ending tag " + s)
