import unittest
from unittest import TestCase

from filter_generator_exception import FilterGeneratorException, raise_cannot_find_tag


class TestFilterGeneratorException(TestCase):
    def test_raise_cannot_find_tag(self):
        with self.assertRaises(FilterGeneratorException):
            raise_cannot_find_tag("somebody", "once.told")
        try:
            raise_cannot_find_tag("somebody", "once.told")
        except FilterGeneratorException as fge:
            self.assertEqual("Cannot find starting tag 'somebody' in 'once.told'.", fge.msg)

        with self.assertRaises(FilterGeneratorException):
            raise_cannot_find_tag("me", "the.world", False)
        try:
            raise_cannot_find_tag("me", "the.world", False)
        except FilterGeneratorException as fge:
            self.assertEqual("Cannot find ending tag 'me' in 'the.world'.", fge.msg)

        with self.assertRaises(FilterGeneratorException):
            raise_cannot_find_tag("is", "gonna.roll", surrounded=False)
        try:
            raise_cannot_find_tag("is", "gonna.roll", surrounded=False)
        except FilterGeneratorException as fge:
            self.assertEqual("Cannot find tag 'is' in 'gonna.roll'.", fge.msg)


if __name__ == "__main__":
    unittest.main()
