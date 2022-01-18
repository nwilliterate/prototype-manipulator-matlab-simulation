function dp = rk(x, u, T)
k1 = plant(x,u)*T;
k2 = plant(x+k1*0.5,u)*T;
k3 = plant(x+k2*0.5,u)*T;
k4 = plant(x+k3,u)*T;
dp = x+((k1+k4)/6+(k2+k3)/3);