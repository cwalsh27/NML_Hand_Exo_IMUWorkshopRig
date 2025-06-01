# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information


project = 'NML Hand Exoskeleton'
copyright = '2025, Neuromechatronics Lab'
author = 'Neuromechatronics Lab'
release = '0.0.2'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'breathe',
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx_rtd_theme'
]

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"
html_static_path = ['_static']
html_logo = "_static/LabLogoRedSquare.png"

import os
import sys

# Make sure Python can find your Python module
sys.path.insert(0, os.path.abspath('../src'))

# Breathe configuration
breathe_projects = {
    "NMLHandExo": "../build/doxygen/xml"
}
breathe_default_project = "NMLHandExo"
