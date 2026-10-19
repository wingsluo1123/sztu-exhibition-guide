"""Positive and negative controls for the offline chair diagnostic itself."""

import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest


spec = importlib.util.spec_from_file_location(
    'chair_projection_diagnostic', Path(__file__).resolve().parents[3]
    / 'tools' / 'verify_chair_ground_projection.py')
diagnostic = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diagnostic)


def captures(root, fault=None):
    for label in diagnostic.STAGES:
        points = [[0.8, 0, -0.68]]  # analytically known floor
        if label in diagnostic.TARGETS and fault != 'missed_chair':
            region = diagnostic.TARGETS[label]
            angle = {'front': 0, 'left': np.pi / 6, 'right': -np.pi / 6}[region]
            points.append([1.2 * np.cos(angle), 1.2 * np.sin(angle), -0.58])
        if fault == 'background_outlier' and label == 'open_baseline':
            points.append([1.1, 0, -0.58])
        name = label + '_frame_0.npz'
        np.savez(root / name, points=np.array(points))
        fits = [{'normal': [0, 0, 1], 'center': [0, 0, -0.68],
                 'frame_path': '/robot/evidence/' + name, 'stamp': 10.0}]
        (root / (label + '_ground_fits.json')).write_text(json.dumps({'fits': fits}))


def test_analytic_floor_and_low_chair_controls_pass(tmp_path):
    captures(tmp_path)
    report = diagnostic.analyze(tmp_path)
    assert report['control_failures'] == []
    for label, target in diagnostic.TARGETS.items():
        metrics = report['stages'][label][0]['regions'][target]
        assert metrics['old_nearest_projected_range_m'] is None
        assert metrics['candidate_nearest_projected_range_m'] == pytest.approx(1.2)
        assert metrics['low_points_recovered'] == 1


@pytest.mark.parametrize('fault', ['missed_chair', 'background_outlier'])
def test_missing_chair_or_background_return_is_not_hidden(tmp_path, fault):
    captures(tmp_path, fault)
    report = diagnostic.analyze(tmp_path)
    if fault == 'missed_chair':
        assert len(report['control_failures']) == 3
    else:
        assert report['control_failures'] == ['open_baseline_frame_0.npz:front']


def test_missing_reference_is_rejected(tmp_path):
    (tmp_path / 'open_baseline_ground_fits.json').write_text('{"fits": []}')
    with pytest.raises(ValueError, match='baseline floor fits'):
        diagnostic.analyze(tmp_path)
