idMin = 100
idMax = 1000;
goodSize = 0;
featureSize = 0;
idVec = [];
depVec = [];
goodIdVec = [];
goodDepVec = [];
for i = idMin : idMax
[mSeed,rSeed,lastMSeed] = testRovioFeature(df,i,0);
if isempty(mSeed) 
    continue;
else
    if (compareSeed(mSeed,rSeed)) ~= 1
        fprintf("Find not same seed : %d,a = %f vs %f, b = %f vs %f, d = %f vs %f, sigma2 : %f vs %f\n",...
            i, ...
            mSeed.a,rSeed.a, ...
            mSeed.b,rSeed.b, ...
            mSeed.d,rSeed.d, ...
            mSeed.sigma2,rSeed.sigma2);
    end
    if seedIsGood(lastMSeed,0.8) == 1 
        goodIdVec = [goodIdVec;i];
        goodDepVec = [goodDepVec;rSeed.d];
        goodSize = goodSize + 1;
    else
        idVec = [idVec;i];
        depVec = [depVec;rSeed.d];
    end
    featureSize = featureSize + 1;
end
 
end
fprintf("AllSize : %d and goodSize : %d\n",featureSize,goodSize);
figure
plot(idVec,depVec,'r*');
hold on
plot(goodIdVec,goodDepVec,'g*');
grid on;