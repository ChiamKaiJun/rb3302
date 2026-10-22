from glob import glob
from setuptools import setup

setup(
    name='rb3302_nav', version='0.2.0', packages=['rb3302_nav'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/rb3302_nav']),
        ('share/rb3302_nav', ['package.xml', 'README.md']),
        ('share/rb3302_nav/config', glob('config/*')),
    ],
    install_requires=['setuptools', 'numpy', 'Pillow', 'PyYAML'],
    entry_points={'console_scripts': ['path_preview = rb3302_nav.path_preview:main']},
)
