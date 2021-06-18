import unittest
from pathlib import Path
from unittest import TestCase


class TestFactoryGenerator(TestCase):
    project_path = Path('assets/test_item_factory/')
    assets_location = 'assets/test_item_factory/subdir/'

    def test_load_factory(self):
        pass

    def test_generate(self):
        pass


if __name__ == "__main__":
    unittest.main()
