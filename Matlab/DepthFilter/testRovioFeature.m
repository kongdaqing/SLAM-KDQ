function [mSeed,rSeed] = testRovioFeature(rovioData,id)
%TESTROVIOFEATURE 此处显示有关此函数的摘要
%   此处显示详细说明
mSeed = [];
rSeed = [];
dfDatas = extractRovioIdData(rovioData,id);
if isempty(dfDatas) 
    fprintf("No this id data!\n");
    return;
end
len = length(dfDatas.id);
if len == 0 
    fprintf("No data input!\n");
    return;
end
seed = SeedInitializeWithRovioData(dfDatas,1);

for i = 1:len - 1
    meas.x = dfDatas.x(i);
    meas.tau2 = dfDatas.tau2(i);
    seed = DepthFilter(seed,meas);
    pause(0.2);
end
mSeed = seed;
rSeed = SeedInitializeWithRovioData(dfDatas,len);
fprintf("Update count: %d\n",len - 1);
end

