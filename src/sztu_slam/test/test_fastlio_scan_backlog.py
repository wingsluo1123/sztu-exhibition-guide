import shutil
import subprocess
from pathlib import Path

import pytest


@pytest.mark.parametrize('stamps,max_age,expected_discarded,pushed', [
    ([9.0, 9.1, 9.2, 9.3, 9.85, 9.95], 0.3, 4, True),
    ([9.0, 9.1], 0.3, 2, True),
    ([9.75, 9.9], 0.25, 0, True),
    ([9.0, 9.1], 0.0, 0, True),
    ([10.01, 10.1], 0.3, 0, False),
    ([], 0.3, 0, False),
])
def test_fastlio_discards_only_expired_scans(
        tmp_path, stamps, max_age, expected_discarded, pushed):
    compiler = shutil.which('g++')
    if compiler is None:
        pytest.skip('C++ compiler required for FAST-LIO backlog regression')
    root = Path(__file__).resolve().parents[3]
    source = tmp_path / 'scan_backlog_test.cpp'
    binary = tmp_path / 'scan_backlog_test'
    timestamps = ', '.join(str(stamp) for stamp in stamps)
    source.write_text(f'''
#include <cassert>
#include <deque>
#include "fastlio_scan_backlog.h"

int main()
{{
    const std::deque<double> originals{{{timestamps}}};
    std::deque<double> stamps = originals;
    std::deque<int> scans;
    for (std::size_t index = 0; index < stamps.size(); ++index)
        scans.push_back(static_cast<int>(index));
    bool pending = {'true' if pushed else 'false'};
    const auto discarded = sztu::discard_expired_scans(
        scans, stamps, pending, 10.0, {max_age});
    assert(discarded == {expected_discarded});
    assert(scans.size() == originals.size() - discarded);
    assert(scans.size() == stamps.size());
    assert(pending == {'false' if expected_discarded else ('true' if pushed else 'false')});
    for (std::size_t index = 0; index < scans.size(); ++index)
    {{
        assert(scans[index] == static_cast<int>(index + discarded));
        assert(stamps[index] == originals[index + discarded]);
    }}
}}
''')
    subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-I', str(root / 'tools/patches'), str(source),
                    '-o', str(binary)], check=True, capture_output=True, text=True)
    subprocess.run([str(binary)], check=True, capture_output=True, text=True)
