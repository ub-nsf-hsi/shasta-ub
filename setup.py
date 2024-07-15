#!/usr/bin/env python

"""The setup script."""

from setuptools import setup, find_packages

with open('HISTORY.rst') as history_file:
    history = history_file.read()

requirements = []

setup_requirements = []

test_requirements = []

setup(
    author="Hemanth Manjunatha",
    author_email='hemanthm2277@gmail.com',
    python_requires='>=3.5',
    classifiers=[
        'Development Status :: 2 - Pre-Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Natural Language :: English',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
    ],
    description="SHaSTA:  A simulator for human and swarm team applications. ",
    install_requires=requirements,
    license="MIT license",
    long_description=history,
    include_package_data=True,
    keywords='shasta',
    name='shasta',
    packages=find_packages(include=['shasta', 'shasta.*']),
    setup_requires=setup_requirements,
    test_suite='tests',
    tests_require=test_requirements,
    url='https://github.com/hemumanju/shasta',
    version='0.1.0',
    zip_safe=False,
)
