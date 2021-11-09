seed = SeedInitialize(30,10,10,10);
for i = 1 : 1: 100
d = 10 + unidrnd(200);
dErr = 0.2 * d;
meas = MeasInitialize(d,dErr);
seed = DepthFilter(seed,meas);
pause(0.1)
end