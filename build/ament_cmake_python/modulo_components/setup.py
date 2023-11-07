from setuptools import find_packages
from setuptools import setup

setup(
    name='modulo_components',
    version='3.2.0',
    packages=find_packages(
        include=('modulo_components', 'modulo_components.*')),
)
