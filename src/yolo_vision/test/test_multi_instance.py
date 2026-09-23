from types import SimpleNamespace

import numpy as np

from yolo_vision.segmentation import select_targets


class Tensor:
    def __init__(self, value):
        self.value = value

    def cpu(self):
        return self

    def numpy(self):
        return np.asarray(self.value)

    def __getitem__(self, item):
        return Tensor(np.asarray(self.value)[item])


def test_select_targets_keeps_all_bowls_and_filters_other_classes():
    masks = np.zeros((3, 4, 5), dtype=np.float32)
    masks[0, 0, 0] = 1
    masks[1, 1, 1] = 1
    masks[2, 2, 2] = 1
    result = SimpleNamespace(
        boxes=SimpleNamespace(cls=Tensor([0, 1, 0]), conf=Tensor([.8, .99, .6])),
        masks=SimpleNamespace(data=Tensor(masks)),
        names={0: 'bowl', 1: 'cup'},
    )
    selected = select_targets(result, 'bowl', (4, 5, 3))
    assert len(selected) == 2
    assert selected[0].confidence == .8
    assert selected[1].confidence == .6
    assert selected[0].mask.sum() == selected[1].mask.sum() == 1


def test_empty_view_returns_zero_targets():
    result=SimpleNamespace(boxes=SimpleNamespace(cls=Tensor([]),conf=Tensor([])),masks=None,names={0:'bowl'})
    assert select_targets(result,'bowl',(4,5,3))==[]


def test_non_target_objects_return_zero_targets():
    result=SimpleNamespace(boxes=SimpleNamespace(cls=Tensor([1]),conf=Tensor([.9])),masks=None,names={0:'bowl',1:'cup'})
    assert select_targets(result,'bowl',(4,5,3))==[]


def test_detected_bowl_without_mask_is_error_not_zero_targets():
    import pytest
    result=SimpleNamespace(boxes=SimpleNamespace(cls=Tensor([0]),conf=Tensor([.9])),masks=None,names={0:'bowl'})
    with pytest.raises(ValueError,match='missing_target_masks'): select_targets(result,'bowl',(4,5,3))
