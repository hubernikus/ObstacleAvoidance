function [ w ] = compute_weights( distMeas, N , distMeas_min, weightType)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

if nargin < 3
    distMeas_min = 0;
end

if nargin < 4
    weightType = 'inverseGamma';
end

if distMeas_min % different to zero
    distMeas = max(0, distMeas-distMeas_min);
end

w = zeros(1,N);

%Gamma(Gamma<1) = 1;
%Gamma = Gamma-1;
switch weightType
    case 'inverseGamma'
        zeroInd = distMeas==0;
        if sum(zeroInd) % one element equal to zero -- avoid null divison
            if sum(zeroInd) > 1
                warning('DS on two boundaries. Collision might not be avoided. \n');
            end
            w(zeroInd) = 1;
        else
            w = 1./distMeas;
        end
        % Normalization
        w = w/sum(w);
    case 'khansari'
       for i=1:N
            ind = 1:N;
            ind(i) = [];
            w(i) = prod(distMeas(ind)./(distMeas(i)+distMeas(ind)));
       end 
       if (round(sum(w),4) ~= 1)
           fprintf('Seeee it was normalized yet... with sum(w)=%2.3f\n', sum(w))
       end
       % Add normalization -- not in original
       w = w/sum(w);
    otherwise
        warning("Unkown method \n");
end

end
