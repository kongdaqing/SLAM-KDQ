function seedNew = DepthFilter(seed,meas)
%% DEPTHFILTER 深度滤波器 -- 特征点逆深度滤波采用Beta * Gaussian模型近似真实的含有外点的直方图分布
%   seed -  存储特征点深度的概率模型Beta*Gaussian
%        -  a : Beta内点数量
%        -  b : Beta外点数量
%        - depthRange : 逆深度范围(1/minDepth - 1/maxDepth ~= 1/minDepth)
%        -  mu: Gaussian均值
%        -  sigma2 : Gaussian方差
%   meas - 当前量测的高斯分布模型
%        -   x  : Gaussian均值
%        - tau2 : Gaussian方差


norm_scale = (seed.sigma2 + meas.tau2)^0.5;
if isnan(norm_scale)
    printf("norm is nan!");
    return 
end
%获得量测值在上一次深度高斯分布中的概率值
measPInLast = normpdf(meas.x,seed.mu,norm_scale);
%当前量测的高斯分布和上一次高斯分布融合后的高斯噪声
s2 = 1/(1/seed.sigma2 + 1/meas.tau2);
%对应的期望
m = s2 * (seed.mu / seed.sigma2 + meas.x / meas.tau2);
%内点对应的概率
C1 = seed.a / (seed.a + seed.b) * measPInLast;
%外点对应的概率
C2 = seed.b / (seed.a + seed.b) / seed.depthRange;
%归一化概率
normC1C2 = C1 + C2;
C1 = C1 / normC1C2;
C2 = C2 / normC1C2;

f = C1 * (seed.a + 1) / (seed.a + seed.b + 1) + C2 * seed.a / (seed.a + seed.b + 1);
e = C1 * (seed.a + 1) * (seed.a + 2) / ((seed.a + seed.b + 1) * (seed.a + seed.b + 2)) ... 
  + C2 * seed.a * (seed.a + 1) / ((seed.a + seed.b + 1) * (seed.a + seed.b + 2));
% 更新种子
muNew = C1 * m + C2 * seed.mu;
seedNew.sigma2 = C1 * (s2 + m * m) + C2 * (seed.sigma2 + seed.mu^2) - muNew^2;
seedNew.mu = muNew;
seedNew.a = (e - f) / (f - e / f);
seedNew.b = seedNew.a * (1 - f) / f;




end

