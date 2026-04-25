"""dimos — A fork of dimensionalOS/dimos.

A modular, extensible framework for building dimensional operating system
components with Python.

Personal fork: using this to learn the internals and experiment with
custom component pipelines.
"""

from importlib.metadata import PackageNotFoundError, version

try:
    __version__ = version("dimos")
except PackageNotFoundError:
    # Package is not installed, likely running from source
    __version__ = "0.0.0.dev0"

__author__ = "dimos contributors"
__license__ = "Apache-2.0"

__all__ = ["__version__", "__author__", "__license__"]
