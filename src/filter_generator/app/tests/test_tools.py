import unittest
from pathlib import Path
from unittest import TestCase

import tools
from filter_generator_exception import FilterGeneratorException


class TestTools(TestCase):
    current_path_copy = tools.current_path

    def tearDown(self) -> None:
        tools.current_path = self.current_path_copy

    def test_get_conf_schema(self):
        # Assert success
        schema = tools.get_conf_schema()
        self.assertTrue(isinstance(schema, dict))
        self.assertDictEqual({
            'project-path':
                {
                    'type': 'string',
                    'minLength': 1
                },
            'factories': {
                'type': 'array',
                'items': {
                    'type': 'object',
                    'properties': {
                        'path': {
                            'type': 'string',
                            'minLength': 1
                        },
                        'items-path': {
                            'type': 'string',
                            'minLength': 1
                        },
                        'filename': {
                            'type': 'string',
                            'minLength': 4
                        },
                        'header-filename': {
                            'type': 'string',
                            'minLength': 3
                        },
                        'excluded-items': {
                            'type': 'array',
                            'items': {
                                'type': 'string',
                                'minLength': 3
                            }
                        }
                    }
                }
            },
            'tags': {
                'type': 'object',
                'properties': {
                    'factory-header': {
                        'type': 'object',
                        'properties': {
                            'includes-start': {
                                'type': 'string'
                            },
                            'includes-end': {
                                'type': 'string'
                            }
                        }
                    },
                    'factory': {
                        'type': 'object',
                        'properties': {
                            'filters-list-start': {
                                'type': 'string'
                            },
                            'filters-list-end': {
                                'type': 'string'
                            },
                            'instance-creation-start': {
                                'type': 'string'
                            },
                            'instance-creation-stop': {
                                'type': 'string'
                            }
                        }
                    },
                    'filter-headers': {
                        'type': 'object',
                        'properties': {
                            'class-name': {
                                'type': 'string'
                            },
                            'class-name-separator': {
                                'type': 'string'
                            }
                        }
                    }
                }
            }
        }, schema)

        # Assert failures
        tools.current_path = Path("bad")
        with self.assertRaises(FilterGeneratorException):
            tools.get_conf_schema()

        try:
            tools.get_conf_schema()
        except FilterGeneratorException as fge:
            self.assertEqual("Cannot find configuration schema file (conf-schema.json)!", fge.msg)

    def test_get_conf(self):
        # Assert success
        conf = tools.get_conf()
        self.assertTrue(isinstance(conf, dict))

        # Assert failures
        tools.current_path = Path("bad")
        with self.assertRaises(FilterGeneratorException):
            tools.get_conf()

        try:
            tools.get_conf()
        except FilterGeneratorException as fge:
            self.assertEqual("Cannot find configuration file (conf.yml)!", fge.msg)

    def test_validate_and_fix_path(self):
        # Assert success
        self.assertEqual(
            Path(self.current_path_copy, "../../proc_image_processing").absolute(),
            tools.validate_and_fix_path(Path("../proc_image_processing"))
        )
        self.assertEqual(
            Path("../../../proc_image_processing"),
            tools.validate_and_fix_path(Path("../../../proc_image_processing"))
        )

        # Assert failures
        with self.assertRaises(FilterGeneratorException):
            tools.validate_and_fix_path("bad/path")

        try:
            tools.validate_and_fix_path("bad/path")
        except FilterGeneratorException as fge:
            self.assertEqual("Cannot find path 'bad/path'.", fge.msg)

    def test_get_files_from_path(self):
        expected_files = ['f1.txt', 'f2.c', 'f3.h', 'f4.h']
        path = Path("assets/test_tools")
        files = tools.get_files_from_path(path)
        self.assertEqual(len(expected_files), len(files))
        for file in files:
            self.assertTrue(file.name in expected_files)

        expected_files = ['f1.txt', 'f2.c', 'f3.h', 'f4.h', 'f5.h', 'f6.c', 'f7.txt', 'f8.h']
        files = tools.get_files_from_path(path, recurse=True)
        self.assertEqual(len(expected_files), len(files))
        for file in files:
            self.assertTrue(file.name in expected_files)

        expected_files = ['f3.h', 'f4.h']
        files = tools.get_files_from_path(path, extension=".h")
        self.assertEqual(len(expected_files), len(files))
        for file in files:
            self.assertTrue(file.name in expected_files)

        expected_files = ['f3.h', 'f4.h', 'f5.h', 'f8.h']
        files = tools.get_files_from_path(path, recurse=True, extension=".h")
        self.assertEqual(len(expected_files), len(files))
        for file in files:
            self.assertTrue(file.name in expected_files)

    def test_validate_and_get_conf(self):
        conf = tools.validate_and_get_conf()
        self.assertTrue(isinstance(conf, dict))
        self.assertEqual(self.current_path_copy.joinpath('../../proc_image_processing'), conf["project-path"])
        self.assertEqual(self.current_path_copy.joinpath('../../proc_image_processing/server'),
                         conf["factories"][0]["path"])
        self.assertEqual(self.current_path_copy.joinpath('../../proc_image_processing/filters'),
                         conf["factories"][0]["items-path"])


if __name__ == "__main__":
    unittest.main()
