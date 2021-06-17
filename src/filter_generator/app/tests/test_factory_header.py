import unittest
from pathlib import Path
from unittest import TestCase

import tools
from factory_header import load
from filter_generator_exception import FilterGeneratorException
from filter_header import FilterHeader


class TestFactoryHeader(TestCase):
    project_path = Path('assets')
    factory1 = Path('assets/subdir/factory1.h')
    bad_factory1 = Path('assets/subdir/bad_factory1.h')
    bad_factory2 = Path('assets/subdir/bad_factory2.h')

    filter_header1 = FilterHeader(Path('assets/test-filter1.h'), 'test-filter1.h', None, 'TestFilter1')
    filter_header2 = FilterHeader(Path('assets/test-filter2.h'), 'test-filter2.h', None, 'TestFilter2')
    bad_filter_header1 = FilterHeader(Path('assets/test-filter1.h'), 'test-filter1.h', None, 'TestFilter1')
    bad_filter_header2 = FilterHeader(Path('assets/test-filter2.h'), 'test-filter2.h', None, 'TestFilter2')
    tags = tools.get_conf()["tags"]["factory-header"]

    def test_load(self):
        factory_header = load(self.project_path, self.factory1, [self.filter_header1, self.filter_header2], self.tags)
        self.assertEqual('factory1.h', factory_header.filename)
        self.assertEqual(2, len(factory_header.included_filters))
        self.assertEqual(self.factory1, factory_header.path)
        self.assertEqual(self.project_path, factory_header.project_path)
        self.assertDictEqual(self.tags, factory_header.tags)
        self.assertEqual([
            '<FILTER_GENERATOR_HEADER_INCLUDES>\n',
            '#include <assets/stuff.h>\n',
            '#include <assets/otherstuff.h>\n',
            '<FILTER_GENERATOR_HEADER_INCLUDES/>'
        ], factory_header.content)

    def test_generate(self):
        # Assert success
        factory_header = self.load_factory(self.factory1)
        factory_header.generate()
        self.assertEqual([
            '<FILTER_GENERATOR_HEADER_INCLUDES>\n',
            '#include <assets/test-filter1.h>\n',
            '#include <assets/test-filter2.h>\n',
            '<FILTER_GENERATOR_HEADER_INCLUDES/>'
        ], factory_header.content)

        # Assert failures
        factory_header = self.load_factory(self.bad_factory1)
        with self.assertRaises(FilterGeneratorException):
            factory_header.generate()

        factory_header = self.load_factory(self.bad_factory1)
        try:
            factory_header.generate()
        except FilterGeneratorException as fge:
            self.assertEqual(
                "Cannot find ending tag '<FILTER_GENERATOR_HEADER_INCLUDES/>' in 'bad_factory1.h'.",
                fge.msg
            )

        factory_header = self.load_factory(self.bad_factory2)
        with self.assertRaises(FilterGeneratorException):
            factory_header.generate()

        factory_header = self.load_factory(self.bad_factory2)
        try:
            factory_header.generate()
        except FilterGeneratorException as fge:
            self.assertEqual(
                "Cannot find starting tag '<FILTER_GENERATOR_HEADER_INCLUDES>' in 'bad_factory2.h'.",
                fge.msg
            )

    def test_write(self):
        factory_header = self.load_factory(self.factory1)
        factory_header.generate()
        factory_header.path = Path('assets/subdir/output.h')
        factory_header.write()
        with open(factory_header.path) as f:
            content = f.readlines()
        self.assertEqual([
            '<FILTER_GENERATOR_HEADER_INCLUDES>\n',
            '#include <assets/test-filter1.h>\n',
            '#include <assets/test-filter2.h>\n',
            '<FILTER_GENERATOR_HEADER_INCLUDES/>'
        ], content)

    def load_factory(self, factory):
        return load(
            self.project_path,
            factory,
            [self.filter_header1, self.filter_header2],
            self.tags
        )


if __name__ == "__main__":
    unittest.main()
