function seed = SeedInitialize(depth,minDepth,a,b)
%% SEEDINITIALIZE 初始化特征点逆深度种子的Beta*Gaussian模型
%      depth - 特征点初始深度
%   minDepth - 特征点可能的最小深度
%          a - Beta分布内点初始数量（a/(a+b)表示内点概率）
%          b - Beta分布外点初始数量
seed.a = a;
seed.b = b;
seed.d = depth;
seed.mu = 1./depth;
seed.depthRange = 1/minDepth;
% sigma2逆深度方差
seed.sigma2 = seed.depthRange^2 / 36; 
end

