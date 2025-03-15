'''
Author: Amadeus
Date: 2025-03-15 15:34:27
LastEditors: Amadeus
LastEditTime: 2025-03-15 15:49:06
FilePath: \PyFlyWheel\setup.py
Description: 
'''
from setuptools import setup, find_packages

setup(
    name="pyflywheel",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        # 在这里列出依赖包
    ],
    author="Xinyao Lun",
    author_email="xyaolun@163.com",
    description="A python module for FlyWheel experiment",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/heyLancee/PyFlyWheel",
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
) 