from setuptools import find_packages
from setuptools import setup

setup(
    name='jackal_helper',
    version='0.0.0',
    packages=find_packages(
        include=('jackal_helper', 'jackal_helper.*')),
)
