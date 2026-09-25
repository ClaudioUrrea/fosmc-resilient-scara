function palatino_style()
%PALATINO_STYLE Figure defaults for the manuscript: Palatino Linotype typeface
%   (falls back to TeX Gyre Pagella / Palatino if not installed), LaTeX-free
%   text, and a true minus sign (U+2212) in tick labels (see fix_minus).
fonts = listfonts;
cand = {'Palatino Linotype', 'TeX Gyre Pagella', 'Palatino', 'URW Palladio L'};
name = 'Times';
for i = 1:numel(cand)
    if any(strcmpi(fonts, cand{i})), name = cand{i}; break; end
end
set(groot, 'defaultAxesFontName', name, 'defaultTextFontName', name, ...
    'defaultLegendFontName', name, 'defaultAxesFontSize', 9, 'defaultTextFontSize', 9, ...
    'defaultAxesTickLabelInterpreter', 'tex', 'defaultTextInterpreter', 'tex', ...
    'defaultLegendInterpreter', 'tex', 'defaultAxesXGrid', 'on', 'defaultAxesYGrid', 'on');
end
