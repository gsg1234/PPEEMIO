%% START OF THE ROUTINE %%
clc; clear;

%% VARIABLES %%
a0=10;
a1=2;
a2=15;
a3=1;

%tita1A=-90*pi/180;
%tita1B=-90*pi/180;

XA=zeros(1,6);
YA=zeros(1,6);
XB=zeros(1,6);
YB=zeros(1,6);

syms x y th tita1A tita1B b1A b1B b2A b2B;
%% BRAS A %%

T1A=transl(a0/2,0,0);
T2A=trotz(tita1A)*transl(a1,0,0);
T3A=trotz(b1A)*transl(a2,0,0);
T4A=trotz(b2A)*transl(a3,0,0);
T5A=trotz(-pi/2);

T1B=trotz(pi)*transl(a0/2,0,0);
T2B=trotz(tita1B)*transl(a1,0,0);
T3B=trotz(b1B)*transl(a2,0,0);
T4B=trotz(b2B)*transl(a3,0,0);
T5B=trotz(pi/2);

Tsol=[cos(th),-sin(th),0,x;
      sin(th),cos(th),0,y;
      0,0,1,0;
      0,0,0,1];

eq1=T1A*T2A*T3A*T4A*T5A-Tsol
eq2=T1B*T2B*T3B*T4B*T5B-Tsol
eq3=b1A-b1B+b2A-b2B+tita1A-tita1B-2*pi



%% BRAS B %%



%% RESOLUTION DES EQUATIONS %%


%% PLOT %%

% P1=T1A(1:2,4);
% XA(2)=P1(1);
% YA(2)=P1(2);
% 
% TAux=T1A*T2A;
% P2=TAux(1:2,4);
% XA(3)=P2(1);
% YA(3)=P2(2);
% 
% TAux=T1A*T2A*T3A;
% P2=TAux(1:2,4);
% XA(3)=P2(1);
% YA(3)=P2(2);
% 
% XA
% YA
