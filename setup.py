#!/usr/bin/env python3
"""
Pi Drone Navigation - Setup Script
"""

from setuptools import setup, find_packages
from pathlib import Path

# Read README
readme_path = Path(__file__).parent / "README.md"
long_description = ""
if readme_path.exists():
    long_description = readme_path.read_text(encoding="utf-8")

# Read requirements
requirements_path = Path(__file__).parent / "requirements.txt"
requirements = []
if requirements_path.exists():
    requirements = [
        line.strip()
        for line in requirements_path.read_text().splitlines()
        if line.strip() and not line.startswith("#")
    ]

setup(
    name="pi-drone-nav",
    version="0.1.0",
    author="Your Name",
    author_email="your.email@example.com",
    description="Autonomous drone navigation for Raspberry Pi with Betaflight",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/yourusername/pi-drone-nav",
    project_urls={
        "Bug Tracker": "https://github.com/yourusername/pi-drone-nav/issues",
        "Documentation": "https://github.com/yourusername/pi-drone-nav#readme",
    },
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Operating System :: POSIX :: Linux",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Topic :: Scientific/Engineering",
        "Topic :: Software Development :: Embedded Systems",
    ],
    package_dir={"": "."},
    packages=find_packages(where="."),
    python_requires=">=3.9",
    install_requires=[
        "pyserial>=3.5",
    ],
    extras_require={
        "full": [
            "flask>=2.0.0",
            "flask-cors>=3.0.0",
            "pymavlink>=2.4.0",
            "pyyaml>=6.0",
        ],
        "dev": [
            "pytest>=7.0.0",
            "pytest-cov>=4.0.0",
            "mypy>=1.0.0",
        ],
    },
    entry_points={
        "console_scripts": [
            "pi-drone-nav=src.main:main",
        ],
    },
    include_package_data=True,
    package_data={
        "": ["config/*.yaml", "config/*.txt", "config/missions/*.json"],
    },
)
