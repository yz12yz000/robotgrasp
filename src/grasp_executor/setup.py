from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

package_name = 'grasp_executor'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/config', glob('config/*.json')),
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
]
# Preserve the directory structure of externally supplied model artifacts.
if Path('models').is_dir():
    for directory in [Path('models'), *sorted(p for p in Path('models').rglob('*') if p.is_dir())]:
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
    entry_points={'console_scripts': ['grasp_executor = grasp_executor.main:main', 'preview_grasp = grasp_executor.debug:offline_main', 'publish_rim_point = grasp_executor.debug:publish_main']},
)
