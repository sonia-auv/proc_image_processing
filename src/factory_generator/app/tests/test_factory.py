import unittest
from pathlib import Path
from unittest import TestCase

import tools
from factory import load, Factory
from factory_generator_exception import FactoryGeneratorException
from item_header import ItemHeader


class TestFactory(TestCase):
    project_path = Path('assets')
    factory1 = Path('assets/test_item_factory/subdir/factory1.cc')
    bad_factory1 = Path('assets/test_item_factory/subdir/bad_factory1.cc')
    bad_factory2 = Path('assets/test_item_factory/subdir/bad_factory2.cc')
    bad_factory3 = Path('assets/test_item_factory/subdir/bad_factory3.cc')
    bad_factory4 = Path('assets/test_item_factory/subdir/bad_factory4.cc')

    item_header1 = ItemHeader(Path('assets/test_item_factory/item1.h'), 'item1.h', None, 'TestItem1')
    item_header2 = ItemHeader(Path('assets/test_item_factory/item2.h'), 'item2.h', None, 'TestItem2')
    tags = tools.get_conf()["tags"]["factory"]
    create_params = ["p1", "p2"]
    item_headers = [item_header1, item_header2]

    def test_load(self):
        # Assert success
        factory = load(self.project_path, self.factory1, self.item_headers, self.create_params, self.tags)
        self.assertTrue(isinstance(factory, Factory))
        self.assertEqual('factory1.cc', factory.filename)
        self.assertEqual(self.factory1, factory.path)
        self.assertEqual(self.project_path, factory.project_path)
        self.assertEqual(self.tags, factory.tags)
        self.assertEqual(self.item_headers, factory.included_item_headers)
        self.assertEqual(self.create_params, factory.create_params)
        self.assertEqual([
            'switch(name){\n',
            '    // <FACTORY_GENERATOR_INSTANCE_CREATION>\n',
            "    case 'AdaptiveThreshold':\n",
            '\t\treturn new AdaptiveThreshold(globalParams);\n',
            "\tcase 'BackgroundSubstract':\n",
            '\t\treturn new BackgroundSubstract(globalParams);\n',
            '    // <FACTORY_GENERATOR_INSTANCE_CREATION/>\n',
            '    default:\n',
            '        return null;\n',
            '}'
        ], factory.content)

    def test_generate_instance_creation(self):
        # Assert success
        factory = load(self.project_path, self.factory1, self.item_headers, self.create_params, self.tags)
        factory.generate_instance_creation()
        self.assertEqual([
            'switch(name){\n',
            '    // <FACTORY_GENERATOR_INSTANCE_CREATION>\n',
            "\tcase 'TestItem1':\n\t\treturn new TestItem1(p1, p2);\n",
            "\tcase 'TestItem2':\n\t\treturn new TestItem2(p1, p2);\n",
            '    // <FACTORY_GENERATOR_INSTANCE_CREATION/>\n',
            '    default:\n',
            '        return null;\n',
            '}'
        ], factory.content)

        # Assert failures
        # Missing starting tag for instance creation
        factory = load(self.project_path, self.bad_factory1, self.item_headers, self.create_params, self.tags)
        with self.assertRaises(FactoryGeneratorException):
            factory.generate_instance_creation()
        try:
            factory.generate_instance_creation()
        except FactoryGeneratorException as fge:
            self.assertEqual(
                "Cannot find starting tag '<FACTORY_GENERATOR_INSTANCE_CREATION>' in 'bad_factory1.cc'.",
                fge.msg
            )

        # Missing ending tag for instance creation
        factory = load(self.project_path, self.bad_factory2, self.item_headers, self.create_params, self.tags)
        with self.assertRaises(FactoryGeneratorException):
            factory.generate_instance_creation()
        factory = load(self.project_path, self.bad_factory2, self.item_headers, self.create_params, self.tags)
        try:
            factory.generate_instance_creation()
        except FactoryGeneratorException as fge:
            self.assertEqual(
                "Cannot find ending tag '<FACTORY_GENERATOR_INSTANCE_CREATION/>' in 'bad_factory2.cc'.",
                fge.msg
            )


if __name__ == "__main__":
    unittest.main()
