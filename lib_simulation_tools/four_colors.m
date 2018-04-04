function c = four_colors(m2)
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

green = [0,1,0];
blue = [0,0,1];
white = [1 1 1];
%orange = [255,140,0]/255;
orange = [255,215,0]/255;
red = [1 0 0];

c = zeros(m2+1,3);

% Green - Blue strip
dCol = (green-blue)/(m2-1);
for ii = 1:m2
    c(ii,:) = blue+dCol*(ii-1);
end

% White middle
c(m2+1,:) = white;

% orange to red
dCol = (red-orange)/(m2-1);

for ii = 1:m2
    c(ii+m2+1,:) = orange+dCol*(ii-1);    
end

