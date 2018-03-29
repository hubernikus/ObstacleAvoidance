function [ h ] = drawQuiverArrow(x,dx, col, pointBased)
% Draw arrows
if nargin < 3
    props = 'b';
end

if nargin == 4
    if pointBased
        dx = dx - x;
    end     
end

quiver(x(1), x(2), dx(1), dx(2),1,'Color', col,'LineWidth',2)

end