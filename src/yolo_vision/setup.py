from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

package_name = 'yolo_vision'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/config', glob('config/*.json')),
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
]
# Preserve the directory structure of the checked-in OpenVINO model artifacts.
model_root = Path('yolo26n-seg_openvino_model')
if model_root.is_dir():
    for directory in [model_root, *sorted(p for p in model_root.rglob('*') if p.is_dir())]:
        files = sorted(str(p) for p in directory.iterdir() if p.is_file())
        if files:
            data_files.append(('share/' + package_name + '/' + directory.as_posix(), files))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=('test', 'tests')),
    data_files=data_files,
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=False,
    maintainer='Robot Grasp Workspace maintainer',
    maintainer_email='maintainer@example.com',
    description='Independent one-shot robot grasp module',
    license='Proprietary',
    entry_points={'console_scripts': ['yolo_vision = yolo_vision.main:main', 'debug_vision = yolo_vision.debug:main']},
)
