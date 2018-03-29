function [ h ] = drawArrow_vel(x,dx, props, figHandle)
% Draw arrows
h = annotation('arrow');

if(nargin < 4)
    figHandle = gca;
end

set(h,'parent', figHandle, ...
    'position', [x(1),x(2),dx(1),dx(2)], ...
    'HeadLength', 5, 'HeadWidth', 5, 'HeadStyle', 'cback1', ...
    props{:});

end