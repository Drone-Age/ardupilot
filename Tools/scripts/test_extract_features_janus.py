#!/usr/bin/env python3
"""Regression checks for Janus feature extraction.

AP_FLAKE8_CLEAN
"""
import re
import unittest

from extract_features import ExtractFeatures


class JanusExtractionTests(unittest.TestCase):
    def test_catalog_is_covered(self):
        ExtractFeatures('unused').validate_features_list()

    def test_exact_update_symbol(self):
        rules = [pattern for name, pattern in ExtractFeatures('unused').features
                 if name == 'AP_CARGO_IMPACT_ENABLED']
        self.assertEqual(len(rules), 1)
        self.assertIsNotNone(re.match(rules[0], 'AP_CargoImpact::update()'))
        self.assertIsNone(re.match(rules[0], 'AP_CargoImpact::update_other()'))
        self.assertIsNone(re.match(rules[0], 'Other::update()'))


if __name__ == '__main__':
    unittest.main()
