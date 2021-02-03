#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Additional non-ros/catkin_ws setup.py to use this package without ros dependencies.

import os
import setuptools

__package_name = "topic_store"


if __name__ == '__main__':
    import xml.etree.ElementTree as ET
    root_path = os.path.join(os.path.abspath(__file__), "../")
    package_path = os.path.abspath(os.path.join(root_path, "../package.xml"))
    requirements_path = os.path.abspath(os.path.join(root_path, "../requirements.txt"))
    extra_requirements_path = os.path.abspath(os.path.join(root_path, "extra_requirements.txt"))
    readme_path = os.path.abspath(os.path.join(root_path, "../docs/README.md"))

    package_xml = ET.parse(str(package_path)).getroot()
    version = package_xml.find("version").text

    with open(requirements_path, "r") as fh:
        requirements = list(map(lambda x: x.strip(), fh.readlines()))
    with open(extra_requirements_path, "r") as fh:
        extra_requirements = list(map(lambda x: x.strip(), fh.readlines()))

    requirements.extend(extra_requirements)

    setuptools.setup(
        name=__package_name,
        version=version,
        author="Raymond Tunstill",
        author_email="ray.tunstill@live.co.uk",
        description="ROS package used for serialising common ROS messages to a database or filesystem.",
        long_description=open(readme_path).read(),
        long_description_content_type="text/markdown",
        url="https://github.com/RaymondKirk/topic_store",
        licence="MIT",
        packages=list(filter(lambda x: __package_name in x, setuptools.find_packages(exclude=["tests"]))),
        classifiers=[
            "Programming Language :: Python :: 2",
            "Programming Language :: Python :: 3",
        ],
        setup_requires=['wheel', "Cython"],
        install_requires=requirements,
        dependency_links=["https://rospypi.github.io/simple/"],  # deprecated: use pip install --extra-index-url
        python_requires='>=2.7',
    )