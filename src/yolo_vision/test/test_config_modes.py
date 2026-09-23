"""Pure configuration inheritance; never import a model or initialize ROS."""
import json
from dataclasses import asdict
from pathlib import Path

import pytest
from yolo_vision.config import VisionConfig, load_config, load_effective_config, resolve_model_path


def write(path, value):
    path.write_text(json.dumps(value))
    return path


@pytest.mark.parametrize('legacy', [None, False, True])
def test_all_targets_config_inherits_roi_and_ignores_legacy_mode(tmp_path, legacy):
    base = write(tmp_path/'base.json', {'input_timeout': 90})
    values = {'inference_roi': [320, 350, 740, 550]}
    if legacy is not None:
        values['multi_instance'] = legacy
    override = write(tmp_path/'roi.json', values)
    originals = [p.read_bytes() for p in (base, override)]
    config = load_effective_config(base, override)
    assert 'multi_instance' not in asdict(config)
    assert config.inference_roi == values['inference_roi']
    assert config.input_timeout == 90
    assert [p.read_bytes() for p in (base, override)] == originals
    assert resolve_model_path(config, '/share/yolo') == Path('/share/yolo/yolo26n-seg_openvino_model')


@pytest.mark.parametrize('bad', [{'multi_instance': 'false'}, {'multi_instance': 0},
    {'multi_instance': None}, {'unknown': 1}, {'confidence': False}, {'input_timeout': -1},
    {'inference_roi': [0, 0, 9000, 1]}, [], None])
@pytest.mark.parametrize('layer', ['base', 'override'])
def test_invalid_layers_rejected(tmp_path, bad, layer):
    base = write(tmp_path/'base.json', bad if layer == 'base' else {'multi_instance': True})
    override = write(tmp_path/'override.json', bad if layer == 'override' else {'multi_instance': True})
    with pytest.raises((ValueError, TypeError)):
        load_effective_config(base, override)


def test_bad_base_cannot_be_hidden_by_override(tmp_path):
    base = write(tmp_path/'base.json', {'multi_instance': 'false'})
    override = write(tmp_path/'override.json', {'multi_instance': True})
    with pytest.raises(ValueError, match='multi_instance'):
        load_effective_config(base, override)


def test_file_errors(tmp_path):
    base = write(tmp_path/'base.json', {})
    missing = tmp_path/'missing.json'
    with pytest.raises(FileNotFoundError):
        load_effective_config(base, missing)
    missing.write_text('{broken')
    with pytest.raises(json.JSONDecodeError):
        load_effective_config(base, missing)
    assert asdict(load_effective_config(base)) == asdict(VisionConfig())


def test_existing_roi_loads_without_object_count_switch():
    configs = Path(__file__).parents[1]/'config'
    roi = configs/'table_roi.example.json'
    assert 'multi_instance' not in asdict(load_config(roi))
    assert 'multi_instance' not in asdict(load_effective_config(configs/'place_config.json', roi))
