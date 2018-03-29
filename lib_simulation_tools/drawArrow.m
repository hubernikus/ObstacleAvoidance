function [ h ] = drawArrow(x,y, props, figHandle)
% Draw arrows
h = annotation('arrow');

if(nargin < 4)
    figHandle = gca;
end

set(h,'parent', figHandle, ...
    'position', [x(1),y(1),x(2)-x(1),y(2)-y(1)], ...
    'HeadLength', 10, 'HeadWidth', 10, 'HeadStyle', 'cback1', ...
    props{:});

end