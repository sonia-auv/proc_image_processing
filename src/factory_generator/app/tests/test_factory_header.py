import unittest
from pathlib import Path
from unittest import TestCase

import tools
from factory_generator_exception import FactoryGeneratorException
from factory_header import load
from item_header import ItemHeader


class TestFactoryHeader(TestCase):
    project_path = Path('assets')
    factory1 = Path('assets/test_item_factory/subdir/factory1.h')
    bad_factory1 = Path('assets/test_item_factory/subdir/bad_factory1.h')
    bad_factory2 = Path('assets/test_item_factory/subdir/bad_factory2.h')

    item_header1 = ItemHeader(Path('assets/test_item_factory/item1.h'), 'item1.h', None, 'TestItem1')
    item_header2 = ItemHeader(Path('assets/test_item_factory/item2.h'), 'item2.h', None, 'TestItem2')
    bad_item_header1 = ItemHeader(Path('assets/test_item_factory/item1.h'), 'item1.h', None, 'TestItem1')
    bad_item_header2 = ItemHeader(Path('assets/test_item_factory/item2.h'), 'item2.h', None, 'TestItem2')
    tags = tools.get_conf()["tags"]["factory-header"]

    def test_load(self):
        factory_header = load(self.project_path, self.factory1, [self.item_header1, self.item_header2], self.tags)
        self.assertEqual('factory1.h', factory_header.filename)
        self.assertEqual(2, len(factory_header.included_item_headers))
        self.assertEqual(self.factory1, factory_header.path)
        self.assertEqual(self.project_path, factory_header.project_path)
        self.assertDictEqual(self.tags, factory_header.tags)
        self.assertEqual([
            '<FACTORY_GENERATOR_HEADER_INCLUDES>\n',
            '#include <assets/stuff.h>\n',
            '#include <assets/otherstuff.h>\n',
            '<FACTORY_GENERATOR_HEADER_INCLUDES/>'
        ], factory_header.content)

    def test_generate(self):
        # Assert success
        factory_header = self.load_factory(self.factory1)
        factory_header.generate()
        self.assertEqual([
            '<FACTORY_GENERATOR_HEADER_INCLUDES>\n',
            '#include <assets/test_item_factory/item1.h>\n',
            '#include <assets/test_item_factory/item2.h>\n',
            '<FACTORY_GENERATOR_HEADER_INCLUDES/>'
        ], factory_header.content)

        # Assert failures
        factory_header = self.load_factory(self.bad_factory1)
        with self.assertRaises(FactoryGeneratorException):
            factory_header.generate()

        factory_header = self.load_factory(self.bad_factory1)
        try:
            factory_header.generate()
        except FactoryGeneratorException as fge:
            self.assertEqual(
                "Cannot find ending tag '<FACTORY_GENERATOR_HEADER_INCLUDES/>' in 'bad_factory1.h'.",
                fge.msg
            )

        factory_header = self.load_factory(self.bad_factory2)
        with self.assertRaises(FactoryGeneratorException):
            factory_header.generate()

        factory_header = self.load_factory(self.bad_factory2)
        try:
            factory_header.generate()
        except FactoryGeneratorException as fge:
            self.assertEqual(
                "Cannot find starting tag '<FACTORY_GENERATOR_HEADER_INCLUDES>' in 'bad_factory2.h'.",
                fge.msg
            )

    def test_write(self):
        factory_header = self.load_factory(self.factory1)
        factory_header.generate()
        factory_header.path = Path('assets/test_item_factory/subdir/output.h')
        factory_header.write()
        with open(factory_header.path) as f:
            content = f.readlines()
        self.assertEqual([
            '<FACTORY_GENERATOR_HEADER_INCLUDES>\n',
            '#include <assets/test_item_factory/item1.h>\n',
            '#include <assets/test_item_factory/item2.h>\n',
            '<FACTORY_GENERATOR_HEADER_INCLUDES/>'
        ], content)

    def load_factory(self, factory):
        return load(
            self.project_path,
            factory,
            [self.item_header1, self.item_header2],
            self.tags
        )


if __name__ == "__main__":
    unittest.main()
