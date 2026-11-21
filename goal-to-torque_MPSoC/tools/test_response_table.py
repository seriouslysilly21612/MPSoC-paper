"""pytest: tools/response_table.py on stand-in summaries."""
import csv
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import response_table as R  # noqa: E402


def summary(d, largest):
    d.mkdir()
    (d / 'timing_summary.txt').write_text('window: track\nlargest t_w + t_s of the R5: 100.5 us (its period 7)\n'
                                          'largest t_w + t_e of the R5 with the control computation: %s us (its '
                                          'period 9: t_w 11.1)\n' % largest)
    return str(d)


def test_the_table_has_the_largest_response_of_each_run(tmp_path, monkeypatch):
    a, b = summary(tmp_path / 'a', '478.5'), summary(tmp_path / 'b', '693.5')
    out = tmp_path / 'r.csv'
    monkeypatch.setattr(sys, 'argv', ['response_table.py', '--out', str(out), '0:pressure800:' + a,
                                      '1:pressure800:' + b])
    assert R.main() == 0
    rows = list(csv.DictReader(open(out)))
    assert [(r['mode'], r['level'], r['response_us']) for r in rows] == [('0', 'pressure800', '478.5'),
                                                                        ('1', 'pressure800', '693.5')]


@pytest.mark.parametrize('item', ['x:idle:/tmp', '0:bad level:/tmp', '0:idle'])
def test_bad_items(item):
    with pytest.raises(ValueError):
        R.parse_item(item)


def test_a_summary_without_the_r5_control_is_refused(tmp_path):
    d = tmp_path / 'p0'
    d.mkdir()
    (d / 'timing_summary.txt').write_text('largest chain: 412 us\n')
    with pytest.raises(ValueError):
        R.largest_response_us(str(d))
