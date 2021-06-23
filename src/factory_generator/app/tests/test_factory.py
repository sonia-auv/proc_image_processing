import unittest
from pathlib import Path
from unittest import TestCase

import tools
from factory import load, Factory
from factory_generator_exception import FactoryGeneratorException
from item_header import ItemHeader


class TestFactory(TestCase):
    project_path = Path('assets')
    assets_location = 'assets/test_item_factory/subdir/'
    factory1 = Path(assets_location + 'factory1.cc')
    factory2 = Path(assets_location + 'factory2.cc')
    factory3 = Path(assets_location + 'factory3.cc')
    bad_factory1 = Path(assets_location + 'bad_factory1.cc')
    bad_factory2 = Path(assets_location + 'bad_factory2.cc')
    bad_factory3 = Path(assets_location + 'bad_factory3.cc')
    bad_factory4 = Path(assets_location + 'bad_factory4.cc')

    item_header1 = ItemHeader(Path('assets/test_item_factory/item1.h'), 'item1.h', None, 'TestItem1')
    item_header2 = ItemHeader(Path('assets/test_item_factory/item2.h'), 'item2.h', None, 'TestItem2')
    tags = tools.get_conf()["tags"]["factory"]
    equality_variable = "name"
    create_params = ["p1", "p2"]
    item_headers = [item_header1, item_header2]

    def test_load(self):
        # Assert success
        factory = load(self.project_path, self.factory1, self.item_headers, self.equality_variable, self.create_params,
                       self.tags)
        self.assertTrue(isinstance(factory, Factory))
        self.assertEqual('factory1.cc', factory.filename)
        self.assertEqual(self.factory1, factory.path)
        self.assertEqual(self.project_path, factory.project_path)
        self.assertEqual(self.tags, factory.tags)
        self.assertEqual(self.item_headers, factory.included_item_headers)
        self.assertEqual(self.create_params, factory.create_params)
        self.assertEqual([
            'should remain\n',
            'switch(name){\n',
            '    // <FACTORY_GENERATOR_INSTANCE_CREATION>\n',
            '\tif(name == "TestItem3000"){\n',
            '\t\treturn new TestItem3000(p1, p2);\n',
            '\t}\n',
            '\telse if(name == "TestItem30"){\n',
            '\t    return new TestItem30(p1, p2);\n',
            '\t}\n',
            '    // <FACTORY_GENERATOR_INSTANCE_CREATION/>\n',
            '    else{\n',
            '        return null;\n',
            '    }\n',
            '}\n',
            'should remain'
        ], factory.content)

    def test_generate_instance_creation(self):
        # Assert success
        factory = load(self.project_path, self.factory1, self.item_headers, self.equality_variable, self.create_params,
                       self.tags)
        factory.generate_instance_creation()
        self.assertEqual([
            'should remain\n',
            'switch(name){\n',
            '    // <FACTORY_GENERATOR_INSTANCE_CREATION>\n',
            '\tif(name == "TestItem1"){\n\t\treturn new TestItem1(p1, p2);\n\t}\n',
            '\telse if(name == "TestItem2"){\n\t\treturn new TestItem2(p1, p2);\n\t}\n',
            '    // <FACTORY_GENERATOR_INSTANCE_CREATION/>\n',
            '    else{\n',
            '        return null;\n',
            '    }\n',
            '}\n',
            'should remain'
        ], factory.content)

        # Assert failures
        # Missing starting tag
        factory = load(self.project_path, self.bad_factory1, self.item_headers, self.equality_variable,
                       self.create_params, self.tags)
        with self.assertRaises(FactoryGeneratorException):
            factory.generate_instance_creation()
        try:
            factory.generate_instance_creation()
        except FactoryGeneratorException as fge:
            self.assertEqual(
                "Cannot find starting tag '<FACTORY_GENERATOR_INSTANCE_CREATION>' in 'bad_factory1.cc'.",
                fge.msg
            )

        # Missing ending tag
        factory = load(self.project_path, self.bad_factory2, self.item_headers, self.equality_variable,
                       self.create_params, self.tags)
        with self.assertRaises(FactoryGeneratorException):
            factory.generate_instance_creation()
        factory = load(self.project_path, self.bad_factory2, self.item_headers, self.equality_variable,
                       self.create_params, self.tags)
        try:
            factory.generate_instance_creation()
        except FactoryGeneratorException as fge:
            self.assertEqual(
                "Cannot find ending tag '<FACTORY_GENERATOR_INSTANCE_CREATION/>' in 'bad_factory2.cc'.",
                fge.msg
            )

    def test_generate_headers_list(self):
        # Assert success
        factory = load(self.project_path, self.factory2, self.item_headers, self.equality_variable, self.create_params,
                       self.tags)
        factory.generate_item_headers_list()
        self.assertEqual([
            'should remain\n',
            'std::string FilterFactory::GetFilterList() {\n',
            '    // <FACTORY_GENERATOR_ITEMS_LIST>\n',
            "\treturn \"TestItem1;\"\n\t\t\"TestItem2\";\n",
            '    // <FACTORY_GENERATOR_ITEMS_LIST/>\n',
            '}\n',
            'should remain'
        ], factory.content)

        # Assert failures
        # Missing starting tag
        factory = load(self.project_path, self.bad_factory3, self.item_headers, self.equality_variable,
                       self.create_params, self.tags)
        with self.assertRaises(FactoryGeneratorException):
            factory.generate_item_headers_list()
        try:
            factory.generate_item_headers_list()
        except FactoryGeneratorException as fge:
            self.assertEqual(
                "Cannot find starting tag '<FACTORY_GENERATOR_ITEMS_LIST>' in 'bad_factory3.cc'.",
                fge.msg
            )

        # Missing ending tag
        factory = load(self.project_path, self.bad_factory4, self.item_headers, self.equality_variable,
                       self.create_params, self.tags)
        with self.assertRaises(FactoryGeneratorException):
            factory.generate_item_headers_list()
        factory = load(self.project_path, self.bad_factory4, self.item_headers, self.equality_variable,
                       self.create_params, self.tags)
        try:
            factory.generate_item_headers_list()
        except FactoryGeneratorException as fge:
            self.assertEqual(
                "Cannot find ending tag '<FACTORY_GENERATOR_ITEMS_LIST/>' in 'bad_factory4.cc'.",
                fge.msg
            )

    def test_generate(self):
        factory = load(self.project_path, self.factory3, self.item_headers, self.equality_variable, self.create_params,
                       self.tags)
        factory.generate()
        self.assertEqual([
            'should remain\n',
            'switch(name){\n',
            '    // <FACTORY_GENERATOR_INSTANCE_CREATION>\n',
            '\tif(name == "TestItem1"){\n\t\treturn new TestItem1(p1, p2);\n\t}\n',
            '\telse if(name == "TestItem2"){\n\t\treturn new TestItem2(p1, p2);\n\t}\n',
            '    // <FACTORY_GENERATOR_INSTANCE_CREATION/>\n',
            '    else{\n',
            '        return null;\n',
            '    }\n',
            '}\n',
            'should remain\n',
            '\n',
            'std::string FilterFactory::GetFilterList() {\n',
            '    // <FACTORY_GENERATOR_ITEMS_LIST>\n',
            "\treturn \"TestItem1;\"\n\t\t\"TestItem2\";\n",
            '    // <FACTORY_GENERATOR_ITEMS_LIST/>\n',
            '}\n',
            'should remain'
        ], factory.content)

    def test_write(self):
        factory = load(self.project_path, self.factory3, self.item_headers, self.equality_variable, self.create_params,
                       self.tags)
        factory.generate()
        factory.path = Path(self.assets_location, 'output.cc')
        factory.write()

        with open(factory.path) as f:
            content = f.readlines()
        self.assertEqual([
            'should remain\n',
            'switch(name){\n',
            '    // <FACTORY_GENERATOR_INSTANCE_CREATION>\n',
            '\tif(name == "TestItem1"){\n',
            '\t\treturn new TestItem1(p1, p2);\n',
            '\t}\n',
            '\telse if(name == "TestItem2"){\n',
            '\t\treturn new TestItem2(p1, p2);\n',
            '\t}\n',
            '    // <FACTORY_GENERATOR_INSTANCE_CREATION/>\n',
            '    else{\n',
            '        return null;\n',
            '    }\n',
            '}\n',
            'should remain\n',
            '\n',
            'std::string FilterFactory::GetFilterList() {\n',
            '    // <FACTORY_GENERATOR_ITEMS_LIST>\n',
            "\treturn \"TestItem1;\"\n",
            "\t\t\"TestItem2\";\n",
            '    // <FACTORY_GENERATOR_ITEMS_LIST/>\n',
            '}\n',
            'should remain'
        ], content)


if __name__ == "__main__":
    unittest.main()
