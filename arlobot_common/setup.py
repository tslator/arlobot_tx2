"""Minimal setup for Arlobot Common"""

from setuptools import setup, find_packages

setup(
    name='arlobot-common',
    version='0.1.0',
    license='proprietary',
    description='Common Classes/Functions',

    author='Tim Slator',
    author_email='',
    url='',

    packages=find_packages(where='src'),
    package_dir={'': 'src'},
)
