clear all
clc
close all


load 'data.txt'
load 'data2.txt'
load 'dataDaemon.txt'


t=1:1:length(data);
t2 = 1:1:length(dataDaemon);
plot(t,data(:,69),t,data2(:,57),t,data(:,72),t,data2(:,60),t2,dataDaemon(:,1))