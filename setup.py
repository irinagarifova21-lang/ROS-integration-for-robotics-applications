#!/usr/bin/env python3

from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="ai-navigation-opencv-ros",
    version="1.0.0",
    author="Irina Garifova",
    author_email="irinagarifova21@example.com",
    description="AI-based autonomous navigation system with OpenCV and ROS integration",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/irinagarifova21-lang/ROS-integration-for-robotics-applications",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    classifiers=[
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Development Status :: 4 - Beta",
        "Intended Audience :: Science/Research",
        "Intended Audience :: Developers",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Scientific/Engineering :: Robotics",
    ],
    python_requires=">=3.8",
    install_requires=[
        "opencv-python>=4.5.0",
        "torch>=1.9.0",
        "torchvision>=0.10.0",
        "ultralytics>=8.0.0",
        "numpy>=1.20.0",
        "scipy>=1.7.0",
        "scikit-image>=0.18.0",
        "scikit-learn>=1.0.0",
        "matplotlib>=3.5.0",
        "seaborn>=0.11.0",
        "pandas>=1.3.0",
        "pyyaml>=5.4.1",
        "tqdm>=4.62.0",
    ],
    extras_require={
        "ros": [
            "catkin-pkg>=1.5.0",
        ],
        "gpu": [
            "torch>=1.9.0[cuda]",
        ],
        "dev": [
            "pytest>=7.0.0",
            "pytest-cov>=4.0.0",
            "black>=22.0.0",
            "flake8>=4.0.0",
            "mypy>=0.950",
        ],
    },
    include_package_data=True,
    zip_safe=False,
)