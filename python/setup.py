# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand Pro 2025 SDK is licensed under Mulan PSL v2.

from setuptools import setup
from setuptools.dist import Distribution


class BinaryDistribution(Distribution):
    def has_ext_modules(self):
        return True


with open("VERSION", "r") as f:
    version = f.read().strip()

setup(
    version=version,
    distclass=BinaryDistribution,
)
