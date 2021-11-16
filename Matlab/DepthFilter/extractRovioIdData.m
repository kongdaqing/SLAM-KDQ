function dfData = extractRovioIdData(rovioData,id,plotEnable)
%EXTRACTROVIOIDDATA 此处显示有关此函数的摘要
%   此处显示详细说明
len = length(rovioData.id);
dfData = [];
dfDep = [];
dfDepTau = [];
dfInverse2DepTau = [];
dfDepNoise = [];
sizeVec = [];
j = 1;
for i = 1 : len
   if rovioData.id(i) == id 
    if j > 1 && rovioData.count(i) < dfData.count(j-1)
        fprintf("Seed reborn and we skip this!\n");
        break;
    end
    dfData.count(j) = rovioData.count(i);
    dfData.id(j) = rovioData.id(i);
    dfData.a(j) = rovioData.a(i);
    dfData.b(j) = rovioData.b(i);
    dfData.mu(j) = rovioData.mu(i);
    dfData.sigma2(j) = rovioData.sigma2(i);
    dfData.depthRange(j) = rovioData.depthRange(i);
    dfData.x(j) = rovioData.x(i);
    dfData.tau2(j) = rovioData.tau2(i);
    dfDepTau = [dfDepTau;rovioData.depTau(i)];
    dfInverseTau = inverseErr2DepthErr(dfData.x(j),dfData.tau2(j)^0.5);
    dfInverse2DepTau = [dfInverse2DepTau;dfInverseTau];
    dfDepNoise = [dfDepNoise;rovioData.zNoise(i)];
    dfDep = [dfDep;1/rovioData.x(i)];
    sizeVec = [sizeVec;j];
    j = j + 1;
   end
end
if isempty(dfDep)
fprintf("Id = %d,no this data!\n",id);
return;
end
if nargin < 3 || plotEnable ~= 1
    return 
end
 
close all;
plot(sizeVec,dfDep,'g*',sizeVec,dfDepTau,'r*',sizeVec,dfInverse2DepTau,'b*',sizeVec,dfDepNoise,'r.');
legend('depth','depErr','inverse2DepthErr','zNoise');
grid on;

end

