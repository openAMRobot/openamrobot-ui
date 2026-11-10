"""Tests for count_test_results.py, the CI zero-executed-test guard.

Standard library only, so CI runs it with
``python3 -m unittest discover -s scripts -p 'test_count_test_results.py'``.
"""

import io
import os
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from unittest.mock import patch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import count_test_results  # noqa: E402

PASSED = '<testcase classname="t" name="{n}"/>'
SKIPPED = '<testcase classname="t" name="{n}"><skipped message="no ROS"/></testcase>'
FAILED = '<testcase classname="t" name="{n}"><failure message="x"/></testcase>'


def _suite(*cases):
    body = ''.join(case.format(n=i) for i, case in enumerate(cases))
    return f'<?xml version="1.0"?><testsuites><testsuite name="s">{body}</testsuite></testsuites>'


class CountTestResultsTest(unittest.TestCase):

    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.build = tmp.name

    def _write(self, relative, text):
        path = os.path.join(self.build, relative)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w') as file:
            file.write(text)

    def _run(self):
        out = io.StringIO()
        with redirect_stdout(out):
            code = count_test_results.main(['count_test_results.py', self.build])
        return code, out.getvalue()

    def test_no_result_files_fails(self):
        code, out = self._run()
        self.assertEqual(code, 1)
        self.assertIn('no JUnit result files', out)

    def test_all_skipped_fails(self):
        self._write('pkg/pytest.xml', _suite(SKIPPED, SKIPPED, SKIPPED, SKIPPED, SKIPPED))
        code, out = self._run()
        self.assertEqual(code, 1)
        self.assertIn('tests total: 5', out)
        self.assertIn('tests skipped: 5', out)
        self.assertIn('tests executed: 0', out)
        self.assertIn('no test was executed', out)

    def test_mixed_results_report_executed_and_skipped_separately(self):
        self._write('pkg_a/pytest.xml', _suite(PASSED, PASSED, SKIPPED))
        self._write('pkg_b/test_results/pkg_b/x.xunit.xml', _suite(PASSED, FAILED, SKIPPED))
        code, out = self._run()
        self.assertEqual(code, 0)
        self.assertIn('result files: 2', out)
        self.assertIn('tests total: 6', out)
        self.assertIn('tests skipped: 2', out)
        self.assertIn('tests executed: 4', out)

    def test_bare_testsuite_root_and_attribute_only_suite(self):
        self._write('a/one.xml', '<testsuite name="s"><testcase name="a"/></testsuite>')
        self._write('b/two.xml', '<testsuites><testsuite name="c" tests="3" skipped="3"/>'
                                 '</testsuites>')
        code, out = self._run()
        self.assertEqual(code, 0)
        self.assertIn('tests total: 4', out)
        self.assertIn('tests skipped: 3', out)
        self.assertIn('tests executed: 1', out)

    def test_broken_xml_is_reported_not_ignored(self):
        self._write('pkg/package.xml', '<package><name>x</name></package>')
        self._write('pkg/broken.xml', '<testsuite')
        code, out = self._run()
        self.assertEqual(code, 1)
        self.assertIn('result files: 0', out)
        self.assertIn('broken.xml', out)
        self.assertIn('FAIL: incomplete XML results', out)


    def test_passing_result_does_not_hide_truncated_xml(self):
        self._write('good/pytest.xml', _suite(PASSED))
        self._write('bad/test_results/pkg/truncated.xml', '<testsuite tests="2"><testcase')
        code, out = self._run()
        self.assertEqual(code, 1)
        self.assertIn('tests executed: 1', out)
        self.assertIn('truncated.xml', out)
        self.assertIn('FAIL: incomplete XML results', out)

    def test_passing_result_does_not_hide_unreadable_xml(self):
        self._write('good/pytest.xml', _suite(PASSED))
        self._write('bad/denied.xml', _suite(PASSED))
        original_parse = count_test_results.ET.parse

        def parse(path):
            if str(path).endswith('denied.xml'):
                raise PermissionError('fixture read denied')
            return original_parse(path)

        with patch.object(count_test_results.ET, 'parse', side_effect=parse):
            code, out = self._run()
        self.assertEqual(code, 1)
        self.assertIn('tests executed: 1', out)
        self.assertIn('denied.xml', out)
        self.assertIn('fixture read denied', out)

    def test_valid_non_junit_xml_does_not_fail_valid_results(self):
        self._write('pkg/package.xml', '<package><name>x</name></package>')
        self._write('pkg/pytest.xml', _suite(PASSED))
        code, out = self._run()
        self.assertEqual(code, 0)
        self.assertIn('result files: 1', out)
        self.assertIn('tests executed: 1', out)

    def test_all_malformed_files_are_named(self):
        self._write('good/pytest.xml', _suite(PASSED))
        self._write('bad/first.xml', '<testsuite')
        self._write('bad/second.xml', '<testsuites>')
        code, out = self._run()
        self.assertEqual(code, 1)
        self.assertIn('first.xml', out)
        self.assertIn('second.xml', out)


if __name__ == '__main__':
    unittest.main()
