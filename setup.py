import os

from setuptools import find_packages, setup

package_name = 'ceai'


def files_under(directory, pattern_dirs=("",)):
    """(install_dir, [files]) pairs for every file directly under `directory`.

    launch/ and config/ are INSTALLED now. They used not to be, which is why
    every run-book in this repo said "always launch by path" -- a standing
    workaround for a packaging omission, not a design.
    """
    entries = []
    for sub in pattern_dirs:
        source = os.path.join(directory, sub) if sub else directory
        if not os.path.isdir(source):
            continue
        names = sorted(
            os.path.join(source, name) for name in os.listdir(source)
            if os.path.isfile(os.path.join(source, name))
        )
        if names:
            install = os.path.join('share', package_name, source)
            entries.append((install, names))
    return entries


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['gui', 'gui.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *files_under('launch', ("", "includes")),
        *files_under('config'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matthew Chua',
    maintainer_email='mattkchua@gmail.com',
    description='Nav2 <-> RPV policy bridge, and the launch files around it',
    license='MIT',
    entry_points={
        'console_scripts': [
            # The bridge. One node, any robot -- `-p robot:=spot`.
            'rpv_bridge = ceai.rpv_bridge:main',
            'finish_listener = ceai.finish_listener:main',
        ],
    },
)
