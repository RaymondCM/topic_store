#!/usr/bin/env bash
cd "$(cd -P -- "$(dirname -- "$0")" && pwd -P)"
set -e

# Ensure you have a file ~/.pypirc
#[distutils]
#index-servers =
#    pypi
#
#[pypi]
#repository: https://upload.pypi.org/legacy/
#username: <username>
#password: <password>

# build and release in a python environment
function build_and_release {
  pip install --extra-index-url https://rospypi.github.io/simple/ -e .
  pip install --upgrade setuptools wheel twine
  rm -r build dist || true
  python setup.py sdist bdist_wheel
  python -m twine upload dist/* || true
  rm -r build dist || true
}

# first copy necessary files
cp ../package.xml ./
cp ../LICENCE ./
cp ../docs/README.md ./
cp ../CHANGELOG.rst ./
cp ../requirements.txt ./

# py3 build
python3.7 -m venv tmp_venv37 --clear
source tmp_venv37/bin/activate
build_and_release
deactivate
rm -r tmp_venv37

# py2 build
python2.7 -m virtualenv tmp_venv27 --clear
source tmp_venv27/bin/activate
build_and_release
deactivate
rm -r tmp_venv27

# delete tmp files
rm package.xml
rm LICENCE
rm README.md
rm CHANGELOG.rst
rm requirements.txt
