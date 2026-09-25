function fix_minus(fig)
%FIX_MINUS Replace hyphen-minus (U+002D) by the true minus sign (U+2212) in the
%   tick labels of every axes of figure FIG. Call after the final axis limits
%   are set and before exporting, e.g.  fix_minus(gcf); exportgraphics(gcf,'f.pdf')
if nargin < 1, fig = gcf; end
ax = findall(fig, 'Type', 'axes');
for a = ax'
    for d = 'XYZ'
        lab = get(a, [d 'TickLabel']);
        if isempty(lab), continue; end
        set(a, [d 'TickLabelMode'], 'manual');
        set(a, [d 'TickLabel'], strrep(cellstr(lab), '-', char(8722)));
    end
end
end
