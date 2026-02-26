%% START OF THE ROUTINE %%
clc; clear;

%% VARIABLES %%
a0=10;
a1=2;
a2=15;
a3=1;

tita1A=-90*pi/180;
tita1B=90*pi/180;

XA=zeros(1,6);
YA=zeros(1,6);
XB=zeros(1,6);
YB=zeros(1,6);

syms x y th b1A b1B b2A b2B;
%% BRAS A %%

T1A=transl(a0,0,0);
T2A=trotz(tita1A)*transl(a1,0,0);
T3A=trotz(b1A)*transl(a2,0,0);
T4A=trotz(b2A)*transl(a3,0,0);
T5A=trotz(pi/2);

T1B=trotz(pi)*transl(a0,0,0);
T2B=trotz(tita1B)*transl(a1,0,0);
T3B=trotz(b1B)*transl(a2,0,0);
T4B=trotz(b2B)*transl(a3,0,0);
T5B=trotz(-pi/2);

Tsol=[cos(th),-sin(th),0,x;
      sin(th),cos(th),0,y;
      0,0,1,0;
      0,0,0,1];


eq1=T1A*T2A*T3A*T4A*T5A-Tsol
eq2=T1B*T2B*T3B*T4B*T5B-Tsol
eq3=b1A-b1B+b2A-b2B+tita1A-tita1B+2*pi

%Test using a0=10, a1=2, a2=9, a3=1
%[a,b,c,d,e,f,g]=F(0,-2,-pi/2,-pi/2,pi/2,0,0,eq1,eq2,eq3)


seed=[0,0,0,0,0,0,0];
sol=F(seed,eq1,eq2,eq3)
sqrt(sum(sol.^2))


%solv=[xaux, yaux, thaux, b1Aaux, b1Baux, b2Aaux, b2Baux]
function sol=F(solv,eq1,eq2,eq3)
    sol=[0,0,0,0,0,0,0];    
    x=solv(1);
    y=solv(2);
    th=solv(3);
    b1A=solv(4);
    b1B=solv(5);
    b2A=solv(6);
    b2B=solv(7);
    sol(1)=subs(eq1(1,4));
    sol(2)=subs(eq1(2,4));
    sol(3)=subs(eq2(1,4));
    sol(4)=subs(eq2(2,4));
    sol(5)=subs(eq1(1,1));
    sol(6)=subs(eq2(2,2));
    sol(7)=subs(eq3);

    
end