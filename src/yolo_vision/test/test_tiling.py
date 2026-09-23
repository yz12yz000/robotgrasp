from types import SimpleNamespace as NS
import numpy as np
import pytest
from yolo_vision.config import VisionConfig
from yolo_vision.segmentation import Segmenter, SegmentationResult, tile_windows, suppress_duplicate_masks


def test_tiles_cover_full_view_including_edges():
    covered = np.zeros((800, 1280), int)
    windows = tile_windows(1280, 800, 640, .5)
    for x0,y0,x1,y1 in windows:
        covered[y0:y1,x0:x1] += 1
    assert len(windows) == 6 and covered.min() >= 1
    assert tile_windows(50, 40, 640, .5) == [(0,0,50,40)]


@pytest.mark.parametrize('count', [0,1,2,3])
def test_same_image_tiles_restore_masks_without_duplicate_objects(count):
    height,width = 100,160
    image = np.zeros((height,width,3), np.uint8)
    image[:,:,0] = np.arange(width)[None,:]
    image[:,:,1] = np.arange(height)[:,None]
    boxes = [(20,20,34,36), (75,40,91,55), (130,70,145,86)][:count]
    full_masks = []
    for x0,y0,x1,y1 in boxes:
        m=np.zeros((height,width),bool);m[y0:y1,x0:x1]=True;full_masks.append(m)
    segmenter = Segmenter.__new__(Segmenter)
    segmenter.config = VisionConfig(inference_tile_size=80,expected_width=width,expected_height=height)
    def predict(crop):
        x0,y0 = map(int,crop[0,0,:2]);h,w=crop.shape[:2]
        results=[]
        for mask in full_masks:
            local=mask[y0:y0+h,x0:x0+w]
            if local.any():
                # A clipped mask may have HIGHER confidence than the full mask.
                score=.99 if local.sum()<mask.sum() else (.7 if w==width else .8)
                results.append(SegmentationResult(local.copy(),'bowl',score))
        return results
    segmenter._predict_image=predict
    results=segmenter.predict_all(image)
    assert len(results)==count
    for mask in full_masks:
        assert any(np.array_equal(r.mask,mask) for r in results)


def test_adjacent_instances_are_not_merged():
    left=np.zeros((30,40),bool);left[5:25,3:20]=True
    right=np.zeros_like(left);right[5:25,20:37]=True
    targets=[SegmentationResult(left,'bowl',.8),SegmentationResult(left.copy(),'bowl',.9),SegmentationResult(right,'bowl',.7)]
    results=suppress_duplicate_masks(targets,.5)
    assert len(results)==2 and results[0].confidence==.9


def test_weak_candidate_needs_distinct_view_support():
    mask=np.ones((20,20),bool)
    weak=SegmentationResult(mask,'bowl',.17)
    assert suppress_duplicate_masks([weak],.5,views=[0],min_support=2,single_confidence=.35)==[]
    assert suppress_duplicate_masks([weak,weak],.5,views=[0,0],min_support=2,single_confidence=.35)==[]
    assert len(suppress_duplicate_masks([weak,weak],.5,views=[0,1],min_support=2,single_confidence=.35))==1


@pytest.mark.parametrize('values', [dict(inference_tile_size=x) for x in (-1,32,True,640.)] +
    [dict(tile_overlap=x) for x in (0,.9,float('nan'),True)] + [dict(tile_mask_iou=1)])
def test_invalid_tile_configuration(values):
    with pytest.raises(ValueError,match='invalid_config'):
        VisionConfig(**values)
