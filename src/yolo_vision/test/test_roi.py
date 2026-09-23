from types import SimpleNamespace
import numpy as np
import pytest
from yolo_vision.config import VisionConfig
from yolo_vision import segmentation as mod


@pytest.mark.parametrize('roi', [[0,0,0,5],[-1,0,20,20],[0,0,1281,800],[0,0,10.5,20],[0,0,10],[0,False,10,20]])
def test_invalid_roi(roi):
    with pytest.raises(ValueError, match='inference_roi'): VisionConfig(inference_roi=roi)


def test_crop_mask_restored_to_full_image(monkeypatch):
    config = VisionConfig(inference_roi=[3,2,8,6])
    arm = mod.Segmenter.__new__(mod.Segmenter)
    arm.config = config
    image = np.arange(10*12*3,dtype=np.uint8).reshape(10,12,3)
    def predict(**kwargs):
        assert np.array_equal(kwargs['source'],image[2:6,3:8])
        return [object()]
    arm.model=SimpleNamespace(predict=predict)
    local=np.zeros((4,5),dtype=bool);local[1:3,2:4]=True
    monkeypatch.setattr(mod,'select_targets',lambda result, cls, shape:[mod.SegmentationResult(local,'bowl',.8),mod.SegmentationResult(~local,'bowl',.7)])
    results=arm.predict_all(image)
    assert len(results)==2
    result=results[0]
    expected=np.zeros((10,12),dtype=bool);expected[3:5,5:7]=True
    assert np.array_equal(result.mask,expected)
    assert result.confidence == .8
    assert results[1].mask[2:6,3:8].sum()==(~local).sum()
    assert not results[1].mask[:2].any()


def test_roi_disabled_preserves_full_frame(monkeypatch):
    arm=mod.Segmenter.__new__(mod.Segmenter);arm.config=VisionConfig()
    image=np.zeros((10,12,3),dtype=np.uint8)
    def predict(**kwargs):
        assert kwargs['source'] is image
        return [object()]
    arm.model=SimpleNamespace(predict=predict)
    target=mod.SegmentationResult(np.ones((10,12),dtype=bool),'bowl',.8)
    monkeypatch.setattr(mod,'select_targets',lambda *args:[target])
    assert arm.predict_all(image)==[target]
