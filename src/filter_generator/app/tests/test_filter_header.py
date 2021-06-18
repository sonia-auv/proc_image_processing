import unittest
from pathlib import Path
from unittest import TestCase

from filter_generator_exception import FilterGeneratorException
from filter_header import load, FilterHeader, load_all


class TestFilterHeader(TestCase):
    filter1 = Path('assets/test_filter_factory/test-filter1.h')
    filter2 = Path('assets/test_filter_factory/test-filter2.h')
    bad_filter1 = Path('assets/test_filter_factory/bad-test-filter1.h')
    bad_filter2 = Path('assets/test_filter_factory/bad-test-filter2.h')

    def test_load(self):
        # Assert success
        loaded = load(self.filter1, {
            "class-name": "FILTER_GENERATOR_CLASS_NAME",
            "class-name-separator": "="
        })
        self.assertTrue(isinstance(loaded, FilterHeader))
        self.assertEqual(self.filter1, loaded.path)
        self.assertEqual('test-filter1.h', loaded.filename)
        self.assertEqual(['// FILTER_GENERATOR_CLASS_NAME=TestFilter1'], loaded.content)
        self.assertEqual('TestFilter1', loaded.class_name)

        # Assert failures
        with self.assertRaises(FilterGeneratorException):
            load(self.filter1, {
                "class-name": "bFILTER_GENERATOR_CLASS_NAME",
                "class-name-separator": "="
            })

        try:
            load(self.filter1, {
                "class-name": "bFILTER_GENERATOR_CLASS_NAME",
                "class-name-separator": "="
            })
        except FilterGeneratorException as fge:
            self.assertEqual("Cannot find tag 'bFILTER_GENERATOR_CLASS_NAME' in 'test-filter1.h'.", fge.msg)

        with self.assertRaises(FilterGeneratorException):
            load(self.filter2, {
                "class-name": "FILTER_GENERATOR_CLASS_NAME",
                "class-name-separator": ":"
            })

        try:
            load(self.filter2, {
                "class-name": "FILTER_GENERATOR_CLASS_NAME",
                "class-name-separator": ":"
            })
        except FilterGeneratorException as fge:
            self.assertEqual("Cannot find tag 'FILTER_GENERATOR_CLASS_NAME:' in 'test-filter2.h'.", fge.msg)

    def test_load_all(self):
        filter_headers_paths = [
            self.filter1,
            self.filter2,
            self.bad_filter1,
            self.bad_filter2
        ]
        excluded_filter_headers = [self.bad_filter2]
        filter_headers = load_all(filter_headers_paths, excluded_filter_headers, {
            "class-name": "FILTER_GENERATOR_CLASS_NAME",
            "class-name-separator": "="
        })
        expected_filenames = ["test-filter1.h", "test-filter2.h"]
        expected_paths = [self.filter1, self.filter2]

        self.assertEqual(2, len(filter_headers))
        for filter_header in filter_headers:
            self.assertTrue(filter_header.filename in expected_filenames)
            self.assertTrue(filter_header.path in expected_paths)


if __name__ == "__main__":
    unittest.main()
