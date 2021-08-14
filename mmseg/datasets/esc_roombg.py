import os.path as osp

from .builder import DATASETS
from .custom import CustomDataset


@DATASETS.register_module()
class ESCRoomDatasetBG(CustomDataset):
    """ESC Room dataset base on Pascal VOC dataset type.

    Args:
        split (str): Split txt file for Pascal VOC.
    """

    CLASSES = ('undrivable', 'roomfloor')

    PALETTE = [[0, 0, 0], [0, 255, 0]]

    def __init__(self, split, **kwargs):
        super(ESCRoomDatasetBG, self).__init__(
            img_suffix='.jpg', seg_map_suffix='.png', split=split, **kwargs)
        assert osp.exists(self.img_dir) and self.split is not None
