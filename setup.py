import pybind11
import numpy as np
from setuptools import setup, Extension

ext_modules = [
    Extension(
        "hacker_league.hacker_league_physics",
        ["python_bindings.cpp"],
        include_dirs=[
            pybind11.get_include(),
            np.get_include(),
            "."
        ],
        extra_compile_args=['-std=c++17', '-O3'],
        language='c++'
    ),
]

setup(
    ext_modules=ext_modules,
    packages=['hacker_league'],
    package_dir={'': 'python'},
)
