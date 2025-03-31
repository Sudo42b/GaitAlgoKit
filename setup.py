from setuptools import setup, find_packages, find_namespace_packages
import sys
import os
import codecs
from pathlib import Path
from typing import List

def read(rel_path):
    here = os.path.abspath(os.path.dirname(__file__))
    with codecs.open(os.path.join(here, rel_path), 'r', 'utf-8') as fp:
        return fp.read()
    

def parse_requirements(filename):
    """Load requirements from a pip requirements file"""
    requirements = []
    dependency_links = []
    
    try:
        filepath = Path(filename)
        if not filepath.exists():
            print(f"Warning: Requirements file {filename} not found")
            return [], []
            
        with codecs.open(filepath, 'r', 'utf-8') as f:
            lines = f.readlines()
            
        for line in lines:
            line = line.strip()
            if not line or line.startswith('#'):
                continue

            # Windows 전용 패키지 제외
            if line.startswith('win-inet-pton'):
                continue
                
            # Intel MKL 관련 패키지 제외
            if any(pkg in line for pkg in ['mkl_fft', 'mkl_random', 'mkl-service']):
                continue

            if line.startswith('--find-links'):
                link = line.split('--find-links')[1].strip()
                dependency_links.append(link)
            elif line.startswith("--extra-index-url"):
                url = line.split("--extra-index-url")[1].strip()
                dependency_links.append(url)
            elif line:
                requirements.append(line)
            
    except Exception as e:
        print(f"Warning: Error parsing requirements.txt: {e}")
        return [], []
        
    return requirements, dependency_links

requirements, dependency_links = parse_requirements('requirements.txt')


# 플랫폼별 추가 의존성
if sys.platform == 'win32':
    platform_specific = [
        "win-inet-pton==1.1.0",
        "mkl-fft",
        "mkl-random==1.2.8",
        "mkl-service==2.4.0",
    ]
else:
    platform_specific = []

try:
    long_description = read('README.md')
except Exception as e:
    print(f"Warning: Error reading README.md: {e}")
    long_description = "Toolkit for gait analysis and exoskeleton control"
from glob import glob
from os.path import basename, splitext

setup(
    name="gaitalgokit",
    version="0.1.0",
    description="Toolkit for gait analysis and exoskeleton control",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="SeonWoo Lee",
    author_email="x21999@naver.com",
    url="https://github.com/Sudo42b/GaitAlgoKit",
    
    # 패키지 디렉토리 구조 설정
    # package_dir={"gaitalgokit": "src"},
    # packages=['gaitalgokit'] + [f"gaitalgokit.{name}" for name in find_packages(where="src")],
    package_dir= {"": "gaitalgokit"},
    packages=find_namespace_packages(where="gaitalgokit"),
    py_modules=[splitext(basename(path))[0] for path in glob('src/*.py')],
    
    # Python 버전 요구사항
    python_requires=">=3.11",
    
    # 의존성 설정
    # install_requires=requirements + platform_specific,
    
    dependency_links=dependency_links,
    
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Science/Research",
        "Intended Audience :: Healthcare Industry",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.11",
        "Topic :: Scientific/Engineering :: Bio-Informatics",
        "Topic :: Scientific/Engineering :: Medical Science Apps.",
    ],

    keywords=["gait analysis", "exoskeleton", "biomechanics", "medical", "healthcare"],
    license="Apache License 2.0",
)

"""
# 개발 모드 설치
pip install -e ".[dev,docs,notebooks]"

# 배포 파일 생성 테스트
python setup.py sdist bdist_wheel

# 패키지 내용 확인
pip show gaitalgokit

# 테스트 실행
pytest tests/
"""