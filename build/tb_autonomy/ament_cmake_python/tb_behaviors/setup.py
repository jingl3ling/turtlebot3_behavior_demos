from setuptools import find_packages
from setuptools import setup

setup(
    name='tb_behaviors',
    version='0.0.0',
    packages=find_packages(
        include=('tb_behaviors', 'tb_behaviors.*')),
)
