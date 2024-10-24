from setuptools import setup, Extension
import pybind11
import numpy as np

ext_modules = [
    Extension(
        "hacker_league",
        ["python_bindings.cpp"],
        include_dirs=[
            pybind11.get_include(),
            np.get_include(),
            "/usr/include/eigen3"
        ],
        extra_compile_args=['-std=c++17', '-O3'],
        language='c++'
    ),
]

setup(
    name="hacker_league",
    ext_modules=ext_modules,
)
