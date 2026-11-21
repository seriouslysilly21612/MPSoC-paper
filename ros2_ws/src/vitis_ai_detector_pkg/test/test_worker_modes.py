"""vitis_ai_worker_yolo: one or two models, and the mode of each request (D-23).

The DPU runtime (xir, vart) is a stand-in here and so is the model: what is tested is which model a
request reaches and what the worker answers. No ROS, no board:
    python3 -m pytest src/vitis_ai_detector_pkg/test/test_worker_modes.py
"""

import io
import json
import sys
import types

import numpy as np
import pytest

for name in ('xir', 'vart'):
    sys.modules.setdefault(name, types.ModuleType(name))
from vitis_ai_detector_pkg import vitis_ai_worker_yolo as W  # noqa: E402


class FakeWorker:
    """a model of input_size that answers with one detection naming itself"""

    made = []

    def __init__(self, model_path, meta_path, log_file=None):
        meta = json.load(open(meta_path))
        self.input_size = meta['input_size']
        self.names = meta.get('names', ['apple', 'banana'])
        self.input_shape = [1, self.input_size, self.input_size, 3]
        self.model_path = model_path
        self.last_timing = {'worker_ms': 1.0}
        FakeWorker.made.append(self)

    def detect(self, image, output_width=None, output_height=None):
        return [
            {
                'class_id': 0,
                'class_name': f'from {self.input_size}',
                'confidence': 1.0,
                'center_x': 0.0,
                'center_y': 0.0,
                'width': 1.0,
                'height': 1.0,
            }
        ]

    def log(self, message):
        pass


def model(tmp_path, name, size, names=None):
    d = tmp_path / name
    d.mkdir()
    meta = {'input_size': size}
    if names is not None:
        meta['names'] = names
    (d / 'decode_meta.json').write_text(json.dumps(meta))
    return str(d / 'net.xmodel'), str(d / 'decode_meta.json')


def test_load_models(tmp_path):
    a = model(tmp_path, 'a', 416)
    b = model(tmp_path, 'b', 320)
    workers, first = W.load_models([a[0], b[0]], [a[1], b[1]], make=FakeWorker)
    assert sorted(workers) == [0, 1] and first == 0 and workers[1].input_size == 320
    workers, first = W.load_models([b[0], a[0]], [b[1], a[1]], make=FakeWorker)
    assert first == 1  # the first given is the default
    one = model(tmp_path, 'odd', 512)
    assert (
        W.load_models([one[0]], [one[1]], make=FakeWorker)[1] == 0
    )  # one model of another size: mode 0


@pytest.mark.parametrize('second', [('c', 416, None), ('d', 320, ['pear']), ('e', 512, None)])
def test_load_models_refuses(tmp_path, second):
    a = model(tmp_path, 'a', 416)
    b = model(tmp_path, *second)
    with pytest.raises(RuntimeError):
        W.load_models([a[0], b[0]], [a[1], b[1]], make=FakeWorker)


def run_main(monkeypatch, argv, requests):
    """the worker's main loop on requests (inline image data); its answers"""
    image = np.zeros((4, 6, 3), np.uint8).tobytes()
    data = b''
    for req in requests:
        req = dict(height=4, width=6, channels=3, data_len=len(image), **req)
        data += json.dumps(req).encode() + b'\n' + image
    out = io.BytesIO()
    monkeypatch.setattr(sys, 'argv', ['worker'] + argv)
    monkeypatch.setattr(sys, 'stdin', types.SimpleNamespace(buffer=io.BytesIO(data)))
    monkeypatch.setattr(sys, 'stdout', types.SimpleNamespace(buffer=out))
    monkeypatch.setattr(W, 'YoloWorker', FakeWorker)
    monkeypatch.setattr(W, 'ShmFrameReader', lambda: None)
    try:
        W.main()
    except SystemExit:
        pass
    return [json.loads(line) for line in out.getvalue().splitlines()]


def test_requests_reach_the_model_of_their_mode(tmp_path, monkeypatch):
    a = model(tmp_path, 'a', 416)
    b = model(tmp_path, 'b', 320)
    answers = run_main(
        monkeypatch,
        ['--model', a[0], '--model', b[0]],
        [{'mode': 1}, {'mode': 0}, {}, {'mode': 3}],
    )
    ready, r1, r0, rdefault, rbad = answers
    assert (
        ready['status'] == 'ready' and ready['modes'] == [0, 1] and ready['input_shape'][1] == 416
    )
    assert r1['mode'] == 1 and r1['detections'][0]['class_name'] == 'from 320'
    assert r0['mode'] == 0 and r0['detections'][0]['class_name'] == 'from 416'
    assert rdefault['mode'] == 0  # no mode in the request: the first model
    assert rbad['error'] and 'not loaded' in rbad['error']


def test_one_model_as_before(tmp_path, monkeypatch):
    a = model(tmp_path, 'a', 416)
    ready, r = run_main(monkeypatch, ['--model', a[0]], [{}])
    assert ready['modes'] == [0] and r['mode'] == 0 and r['error'] is None


def test_a_refused_pair_is_reported(tmp_path, monkeypatch):
    a = model(tmp_path, 'a', 416)
    b = model(tmp_path, 'b', 416)
    (answer,) = run_main(monkeypatch, ['--model', a[0], '--model', b[0]], [])
    assert answer['status'] == 'error' and 'second model of mode 0' in answer['error']
