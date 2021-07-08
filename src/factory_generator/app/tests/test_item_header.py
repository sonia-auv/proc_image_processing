import unittest
from pathlib import Path
from unittest import TestCase

from factory_generator_exception import FactoryGeneratorException
from item_header import load, ItemHeader, load_all


class TestFilterHeader(TestCase):
    item1 = Path('assets/test_item_factory/item1.h')
    item2 = Path('assets/test_item_factory/item2.h')
    bad_item1 = Path('assets/test_item_factory/bad-item1.h')
    bad_item2 = Path('assets/test_item_factory/bad-item2.h')

    def test_load(self):
        # Assert success
        loaded = load(self.item1, {
            "class-name": "FACTORY_GENERATOR_CLASS_NAME",
            "class-name-separator": "="
        })
        self.assertTrue(isinstance(loaded, ItemHeader))
        self.assertEqual(self.item1, loaded.path)
        self.assertEqual('item1.h', loaded.filename)
        self.assertEqual(['// FACTORY_GENERATOR_CLASS_NAME=TestItem1'], loaded.content)
        self.assertEqual('TestItem1', loaded.class_name)

        # Assert failures
        with self.assertRaises(FactoryGeneratorException):
            load(self.item1, {
                "class-name": "bFACTORY_GENERATOR_CLASS_NAME",
                "class-name-separator": "="
            })

        try:
            load(self.item1, {
                "class-name": "bFACTORY_GENERATOR_CLASS_NAME",
                "class-name-separator": "="
            })
        except FactoryGeneratorException as fge:
            self.assertEqual("Cannot find tag 'bFACTORY_GENERATOR_CLASS_NAME' in 'item1.h'.", fge.msg)

        with self.assertRaises(FactoryGeneratorException):
            load(self.item2, {
                "class-name": "FACTORY_GENERATOR_CLASS_NAME",
                "class-name-separator": ":"
            })

        try:
            load(self.item2, {
                "class-name": "FACTORY_GENERATOR_CLASS_NAME",
                "class-name-separator": ":"
            })
        except FactoryGeneratorException as fge:
            self.assertEqual("Cannot find tag 'FACTORY_GENERATOR_CLASS_NAME:' in 'item2.h'.", fge.msg)

    def test_load_all(self):
        item_headers_paths = [
            self.item1,
            self.item2,
            self.bad_item1,
            self.bad_item2
        ]
        excluded_item_headers = [self.bad_item2]
        item_headers = load_all(item_headers_paths, excluded_item_headers, {
            "class-name": "FACTORY_GENERATOR_CLASS_NAME",
            "class-name-separator": "="
        })
        expected_filenames = ["item1.h", "item2.h"]
        expected_paths = [self.item1, self.item2]

        self.assertEqual(2, len(item_headers))
        for item_header in item_headers:
            self.assertTrue(item_header.filename in expected_filenames)
            self.assertTrue(item_header.path in expected_paths)


if __name__ == "__main__":
    unittest.main()
