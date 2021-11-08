function meas = MeasInitialize(depth,depthErr)
%MEASINITIALIZE 此处显示有关此函数的摘要
%   此处显示详细说明

meas.x = 1/depth;
tau = 0.5 * (1/max(1e-7,depth - depthErr) - 1.0/(depth + depthErr));
meas.tau2 = tau^2;
end

