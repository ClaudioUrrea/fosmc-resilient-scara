"""Common figure style: Palatino-type typeface (TeX Gyre Pagella, metric clone of
Palatino Linotype) for text and math, true minus sign U+2212, embedded fonts."""
import glob
import matplotlib
from matplotlib import font_manager

import os
_here = os.path.dirname(os.path.abspath(__file__))
# TrueType conversions of TeX Gyre Pagella (fonts/*.ttf) embed cleanly as Type 42;
# on Windows, 'Palatino Linotype' is used directly if installed.
font_manager.fontManager.ttflist = [e for e in font_manager.fontManager.ttflist
                                    if not ('Pagella' in e.name and e.fname.lower().endswith('.otf'))]
for f in glob.glob(os.path.join(_here, 'fonts', 'texgyrepagella*.ttf')):
    font_manager.fontManager.addfont(f)

FAMILY = ['Palatino Linotype', 'TeX Gyre Pagella', 'Palatino', 'URW Palladio L', 'serif']


def apply():
    matplotlib.rcParams.update({
        'font.family': 'serif', 'font.serif': FAMILY, 'font.size': 8.5,
        'mathtext.fontset': 'custom', 'mathtext.rm': 'TeX Gyre Pagella',
        'mathtext.it': 'TeX Gyre Pagella:italic', 'mathtext.bf': 'TeX Gyre Pagella:bold',
        'mathtext.sf': 'TeX Gyre Pagella', 'mathtext.fallback': 'stix',
        'axes.unicode_minus': True,            # tick labels use U+2212
        'axes.formatter.use_mathtext': True,
        'pdf.fonttype': 42, 'ps.fonttype': 42,
        'axes.grid': True, 'grid.alpha': 0.3, 'savefig.dpi': 300, 'legend.fontsize': 7,
    })
