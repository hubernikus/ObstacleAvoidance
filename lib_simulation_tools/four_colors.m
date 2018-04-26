function c = four_colors(m2, cols_pos, cols_neg,  col_mean)
%REDBLUE    Shades of red and blue color map
%   REDBLUE(M), is an M-by-3 matrix that defines a colormap.
%   The colors begin with bright blue, range through shades of
%   blue to white, and then through shades of red to bright red.
%   REDBLUE, by itself, is the same length as the current figure's
%   colormap. If no figure exists, MATLAB creates one.
%
%   For example, to reset the colormap of the current figure:
%
%             colormap(redblue)
%
%   See also HSV, GRAY, HOT, BONE, COPPER, PINK, FLAG, 
%   COLORMAP, RGBPLOT.

m2 = max([2, m2]);

if nargin<2
    cols_neg(1,:) = [0,0,1]; % Blue
    cols_neg(2,:) = [0,1,0]; % Green

    cols_pos(1,:) = [255,215,0]/255; % orange
    cols_pos(2,:) = [1 0 0]; % red
end
if nargin<4
    col_mean = [1 1 1]; % White
end

c = zeros(2*m2+1,3);

% Green - Blue strip
dCol = (cols_neg(2,:)-cols_neg(1,:) )/(m2-1);
for ii = 1:m2
    c(ii,:) = cols_neg(1,:)+dCol*(ii-1);
end

% White middle
c(m2+1,:) = col_mean;

% orange to red
dCol = (cols_pos(2,:)-cols_pos(1,:) )/(m2-1);
for ii = 1:m2
    c(ii+m2+1,:) = cols_pos(1,:) +dCol*(ii-1);    
end

