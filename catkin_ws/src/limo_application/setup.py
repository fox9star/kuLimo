#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['limo_application'], # 이곳에 실제 파이썬 모듈 폴더 이름을 명시합니다 (없다면 비워둬도 됨)
    package_dir={'': 'src'}       # 만약 파이썬 모듈이 src/limo_application 안에 있다면 이렇게 설정
)

setup(**setup_args)