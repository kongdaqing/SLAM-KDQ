function dfData = extractRovioIdData(rovioData,id)
%EXTRACTROVIOIDDATA 此处显示有关此函数的摘要
%   此处显示详细说明
len = length(rovioData.id);
dfData = [];
j = 1;
for i = 1 : len
   if rovioData.id(i) == id 
    dfData.id(j) = rovioData.id(i);
    dfData.a(j) = rovioData.a(i);
    dfData.b(j) = rovioData.b(i);
    dfData.mu(j) = rovioData.mu(i);
    dfData.sigma2(j) = rovioData.sigma2(i);
    dfData.depthRange(j) = rovioData.depthRange(i);
    dfData.count(j) = rovioData.count(i);
    dfData.x(j) = rovioData.x(i);
    dfData.tau2(j) = rovioData.tau2(i);
    j = j + 1;
   end
end

end

