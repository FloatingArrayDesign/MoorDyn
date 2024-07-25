# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# http://www.sphinx-doc.org/en/master/config

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import subprocess
import shutil
import os
import sys
sys.path.insert(0, os.path.abspath('../wrappers/python/moordyn/'))


# -- Doxygen and breathe steps -----------------------------------------------------------

# Check if we're running on Read the Docs' servers
read_the_docs_build = os.environ.get('READTHEDOCS', None) == 'True'

breathe_projects = {}

os.makedirs("_build/doxygen", exist_ok=True)
if not read_the_docs_build:
    subprocess.call('make clean', shell=True)
subprocess.call('doxygen', shell=True)
os.makedirs("_build/doxygen/out/doxygen", exist_ok=True)
if os.path.exists('_build/doxygen/out/doxygen/html'):
    shutil.rmtree('_build/doxygen/out/doxygen/html')
shutil.move('_build/doxygen/html', '_build/doxygen/out/doxygen/html')

breathe_projects['MoorDyn'] = "_build/doxygen/xml"
breathe_default_project = "MoorDyn"

# breathe_projects = { "trianglelib": "../../doxygen/build/xml/" }
# breathe_default_project = "trianglelib"


# -- Project information -----------------------------------------------------

project = 'MoorDyn'
copyright = '2024, National Renewable Energy Laboratory'
author = 'Matt Hall'

# The full version, including alpha/beta/rc tags
release = '2.3.1' # TODO: can we automate this?


# -- General configuration ---------------------------------------------------

master_doc = 'index'

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
# extensions = ['recommonmark']
# extensions = ['sphinx.ext.autodoc', 'sphinx.ext.napoleon']
extensions = ["breathe", 'sphinx.ext.autodoc',
              'sphinx.ext.napoleon', 'sphinx.ext.mathjax']

napoleon_include_init_with_doc = True

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
# need to run pip install sphinx_rtd_theme to use locally
html_theme = 'sphinx_rtd_theme'
html_theme_options = {
    'collapse_navigation': False
}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
# html_static_path = ['_static']

# specify custom attributes to accept (see https://github.com/sphinx-doc/sphinx/issues/2682)
cpp_id_attributes = ['DECLDIR']

# Keep the doxygen documentaion in readthedocs
html_extra_path = ['_build/doxygen/out']
