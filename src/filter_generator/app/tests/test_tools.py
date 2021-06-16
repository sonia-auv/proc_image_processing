import unittest
from pathlib import Path
from unittest import TestCase

from filter_generator.tools import get_conf, current_path


class TestTools(TestCase):

    def test_get_config(self):
        self.assertDictEqual(get_conf(), {
            "filters-path": Path(current_path, '../../proc_image_processing/filters'),
            "factory-path": Path(current_path, '../../proc_image_processing/server')
        })
        print()


if __name__ == "__main__":
    unittest.main()
