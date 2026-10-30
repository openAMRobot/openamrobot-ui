#!/usr/bin/env python3
"""Count executed tests in colcon's JUnit results and fail when none ran.

colcon test writes one JUnit XML file per test run under the build base
(``ros2/build``); ``colcon test-result`` reads the same files. Its summary
line counts skipped tests in the total, so a run where every test was
skipped still reports a positive count. This script parses the XML files
itself and reports total, skipped and executed separately. Executed is
total minus skipped. It exits 1 when no result file is found or no test
was executed (AGENTS.md: a suite that executes zero tests fails).

Usage: python3 scripts/count_test_results.py [build_dir]   (default: build)
"""

import os
import sys
import xml.etree.ElementTree as ET


def _suites(root):
    if root.tag == 'testsuite':
        return [root]
    if root.tag == 'testsuites':
        return root.findall('testsuite')
    return []


def _count_suite(suite):
    """Return (total, skipped) for one <testsuite>."""
    cases = suite.findall('.//testcase')
    if cases:
        skipped = sum(1 for case in cases if case.find('skipped') is not None)
        return len(cases), skipped
    # A suite without <testcase> elements reports only attributes.
    total = int(suite.get('tests', 0) or 0)
    skipped = int(suite.get('skipped', suite.get('disabled', 0)) or 0)
    return total, skipped


def count(build_dir):
    """Return {'files', 'total', 'skipped', 'executed'} for build_dir."""
    files = total = skipped = 0
    for folder, _dirs, names in os.walk(build_dir):
        for name in sorted(names):
            if not name.endswith('.xml'):
                continue
            try:
                root = ET.parse(os.path.join(folder, name)).getroot()
            except ET.ParseError:
                continue
            suites = _suites(root)
            if not suites:
                continue  # not a JUnit file (colcon test-result skips these too)
            files += 1
            for suite in suites:
                suite_total, suite_skipped = _count_suite(suite)
                total += suite_total
                skipped += suite_skipped
    return {'files': files, 'total': total, 'skipped': skipped, 'executed': total - skipped}


def main(argv):
    build_dir = argv[1] if len(argv) > 1 else 'build'
    result = count(build_dir)
    print(f"result files: {result['files']}")
    print(f"tests total: {result['total']}")
    print(f"tests skipped: {result['skipped']}")
    print(f"tests executed: {result['executed']}")
    if result['files'] == 0:
        print(f'FAIL: no JUnit result files under {build_dir}')
        return 1
    if result['executed'] <= 0:
        print('FAIL: no test was executed (skipped tests do not count)')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
