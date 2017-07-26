import os
import re
import sys

# pip3 install sphinx-pypi-upload
# python3 setup.py build_sphinx
# sphinx-build -b html ../pyftdi/pyftdi/doc .

topdir = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                      os.pardir, os.pardir))
sys.path.append(topdir)


def read(where, *parts):
    """
    Build an absolute path from *parts* and and return the contents of the
    resulting file.  Assume UTF-8 encoding.
    """
    with open(os.path.join(where, *parts), 'rt') as f:
        return f.read()


def find_meta(meta):
    """
    Extract __*meta*__ from meta_file.
    """
    meta_match = re.search(
        r"^__{meta}__ = ['\"]([^'\"]*)['\"]".format(meta=meta),
        meta_file, re.M
    )
    if meta_match:
        return meta_match.group(1)
    raise RuntimeError("Unable to find __{meta}__ string.".format(meta=meta))


meta_file = read(topdir, 'pyftdi', '__init__.py')

version = find_meta('version')

needs_sphinx = '1.6'
extensions = ['sphinx.ext.autodoc',
              'sphinx.ext.doctest']
templates_path = ['templates']
source_suffix = '.rst'
master_doc = 'index'
project = find_meta('title')
contact = '%s <%s>' % (find_meta('author'), find_meta('email'))
copyright = '2017, %s' % contact
show_authors = True

html_theme = 'sphinx_rtd_theme'
htmlhelp_basename = 'doc'

preamble = r'''
\usepackage{wallpaper}
\usepackage{titlesec}

\titleformat{\chapter}[display]{}{\filleft\scshape\chaptername\enspace\thechapter}{-2pt}{\filright \Huge \bfseries}[\vskip4.5pt\titlerule]
\titleformat{name=\chapter, numberless}[block]{}{}{0pt}{\filright \Huge \bfseries}[\vskip4.5pt\titlerule]

\titlespacing{\chapter}{0pt}{0pt}{1cm}
'''

latex_elements = {
  'papersize': 'a4paper',
  'fncychap': '',  # No Title Page
  'releasename': '',
  'sphinxsetup': 'hmargin={2.0cm,2.0cm}, vmargin={2.5cm,2.5cm}, marginpar=5cm',
  'classoptions': ',openany,oneside',  # Avoid blank page aftre TOC, etc.
  'preamble': preamble,
  'releasename': ''
}

latex_documents = [
  ('index', '%s.tex' % project.lower(),
   '%s Documentation' % project,
   contact, u'manual'),
]

# For "manual" documents, if this is true, then toplevel headings are parts,
# not chapters.
latex_toplevel_sectioning = "chapter"

man_pages = [
  ('index', project,
   '%s Documentation' % project,
   [contact], 1)
]

texinfo_documents = [
  ('index', project,
   '%s Documentation' % project,
   contact, '',
   '%s Documentation' % project,
   'Miscellaneous'),
]


def setup(app):
    app.add_stylesheet('https://fonts.googleapis.com/css?family=Raleway')
