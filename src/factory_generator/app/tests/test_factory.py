import unittest
from pathlib import Path
from unittest import TestCase

import tools
from factory import load, Factory
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

    def test_load(self):
        # Assert success
        create_params = ["p1", "p2"]
        item_headers = [self.item_header1, self.item_header2]
        factory = load(self.project_path, self.factory1, item_headers, create_params, self.tags)
        self.assertTrue(isinstance(factory, Factory))
        self.assertEqual('factory1.cc', factory.filename)
        self.assertEqual(self.factory1, factory.path)
        self.assertEqual(self.project_path, factory.project_path)
        self.assertEqual(self.tags, factory.tags)
        self.assertEqual(item_headers, factory.included_item_headers)
        self.assertEqual(create_params, factory.create_params)
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
        pass
        # item_headers = [self.item_header1, self.item_header2]
        # factory = load(self.project_path, self.factory1, item_headers, self.tags)
        # factory.generate_instance_creation()
        # # with open('assets/test_item_factory/subdir/expected_factory1_generate_instance.cc', 'w') as f:
        # #     f.write("".join(factory.content))
        # with open('assets/test_item_factory/subdir/expected_factory1_generate_instance.cc') as f:
        #     content = f.readlines()
        # self.assertEqual(content, factory.content)


if __name__ == "__main__":
    unittest.main()
