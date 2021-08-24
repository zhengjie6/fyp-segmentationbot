## Prerequisites (Tested)
The following are some software requirements that is tested for this program.

- Linux Ubuntu 18.04 (Windows not recommended)
- Python 3.6+
- PyTorch 1.8.0
- CUDA 11.1.1
- mmcv-full 1.3.9
- GCC 5+

Hardware requirements (Recommended)

- Intel Core i5 8th Gen or higer
- 16GB DDR4 RAM or higher
- NVIDIA GeForce RTX2070 or higher (Recommended to get GPU of larger VRAM to train with large crop size. Older GPU may still work but not as well.)
- 512GB SSD

## Installation

a. Create a conda virtual environment and install the relevant dependencies.
Most of the dependencies should be automatically installed. (You may encourter installation issue with mmcv-full)
You may choose to use other version accordingly.
```shell
git clone  https://github.com/zhengjie6/fyp-segmentationbot.git
cd fyp-segmentationbot/requirements

conda env create -f mmseg.yml
conda active mmseg
```

b. Install mmcv-full for Linux
```shell
pip install mmcv-full==1.3.9 -f https://download.openmmlab.com/mmcv/dist/cu111/torch1.8.0/index.html
```

c. Install relevant dependencies and make data directory
```shell
cd mmsegmentation
pip install -e .  # or "python setup.py develop"

mkdir data
ln -s $DATA_ROOT data
```
Note:
By running `pip install -e.`, it installs all minimally required dependencies. To use optional dependencies like `cityscapessripts`  either install them manually with `pip install -r requirements/optional.txt` or specify desired extras when calling `pip` (e.g. `pip install -e .[optional]`). Valid keys for the extras field are: `all`, `tests`, `build`, and `optional`.

d. Install ROS and Other ROS related dependencies
This step is optional, but good to have for ease of troubleshooting. The segmentation computer only requires `rospkg==1.3.0` (installed when running step a) as it only needs to publish and subscribe to ros topics. However, having ROS installed will provide access to developing ros packages in the same computer. InstallingROS debugging tools such as RVIZ may also be useful for debugging.

Follow the instructions at [ROS PAGE] (http://wiki.ros.org/melodic/Installation/Ubuntu) to install ROS Melodic (for Ubuntu 18.04).


## Verification (Credits: MMSegmentation)

To verify whether MMSegmentation and the required environment are installed correctly, we can run sample python codes to initialize a detector and inference a demo image:

```python
from mmseg.apis import inference_segmentor, init_segmentor
import mmcv

config_file = 'configs/pspnet/pspnet_r50-d8_512x1024_40k_cityscapes.py'
checkpoint_file = 'checkpoints/pspnet_r50-d8_512x1024_40k_cityscapes_20200605_003338-2966598c.pth'

# build the model from a config file and a checkpoint file
model = init_segmentor(config_file, checkpoint_file, device='cuda:0')

# test a single image and show the results
img = 'test.jpg'  # or img = mmcv.imread(img), which will only load it once
result = inference_segmentor(model, img)
# visualize the results in a new window
model.show_result(img, result, show=True)
# or save the visualization results to image files
# you can change the opacity of the painted segmentation map in (0, 1].
model.show_result(img, result, out_file='result.jpg', opacity=0.5)

# test a video and show the results
video = mmcv.VideoReader('video.mp4')
for frame in video:
   result = inference_segmentor(model, frame)
   model.show_result(frame, result, wait_time=1)
```

The above code is supposed to run successfully upon you finish the installation.
