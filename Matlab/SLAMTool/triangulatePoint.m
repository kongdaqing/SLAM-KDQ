function [flg,depth,B,detA] = triangulatePoint(R,t,pInRef,pInCur)
%TRIANGULATEPOINT 三角化特征点
% R -- 参考帧到当前帧的旋转矩阵
% t -- 参考帧到当前帧的平移矩阵
% pInRef -- 参考帧上的像素点坐标
% pInCur -- 当前帧上的像素点坐标
A = zeros(3,2); 
A(:,1) = R * pInRef;
A(:,2) = pInCur;
AtA = A' * A;
detA = det(AtA);
depth = [];
B = [];
fprintf("Det(AtA) = %e\n",detA); 
if detA < 1e-6 
   flg = 0;
   return;
end
B = - inv(AtA) * A';
depth = B * t;
flg = 1;
end

