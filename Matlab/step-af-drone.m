clear;close all; clc;
data          = load('pitch_step.txt');
v= data(:,1);
u=data(:,2);
t=data(:,3);

plot(t(5550:7273),(v(5550:7273)+2)*(-1))

v=(v(5550:7273)+2);

t=t(5550:7273);
t=t-21521529;
t=t+1771360
u=u(5550:7273)


plot(t,v)
s=tf('s');
g=-150*(1+0.001*s)/s
step(g*tf4/(1+g*tf4))
