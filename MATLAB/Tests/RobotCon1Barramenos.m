%% START OF THE ROUTINE %%
clc; clear;

%% VARIABLES %%
a0=10;
a1=5;
a2=sqrt(2)*(a0+a1);

tita1A=0;
tita1B=0;

XA=zeros(1,6);
YA=zeros(1,6);
XB=zeros(1,6);
YB=zeros(1,6);

syms x y th b1A b1B;
%% BRAS %%

T1A=transl(a0,0,0);
T2A=trotz(tita1A)*transl(a1,0,0);
T3A=trotz(b1A)*transl(a2,0,0);
T4A=trotz(-pi/2-tita1A-b1A);

T1B=trotz(pi)*transl(a0,0,0);
T2B=trotz(tita1B)*transl(a1,0,0);
T3B=trotz(b1B)*transl(a2,0,0);
T4B=trotz(pi/2-tita1B-b1B);

Tsol=[cos(th),-sin(th),0,x;
      sin(th),cos(th),0,y;
      0,0,1,0;
      0,0,0,1];


eq1=T1A*T2A*T3A*T4A-Tsol
eq2=T1B*T2B*T3B*T4B-Tsol


seed=[0,-(a0+a1),-pi/2,-pi/4,pi/4];
sol=F(seed,eq1,eq2)
sqrt(sum(sol.^2))


%solv=[xaux, yaux, thaux, b1Aaux, b1Baux]
function sol=F(solv,eq1,eq2)
    sol=[0,0,0,0,0];    
    x=solv(1);
    y=solv(2);
    th=solv(3);
    b1A=solv(4);
    b1B=solv(5);
    sol(1)=subs(eq1(1,4));
    sol(2)=subs(eq1(2,4));
    sol(3)=subs(eq2(1,4));
    sol(4)=subs(eq2(2,4));
    sol(5)=subs(eq1(1,1));

    
end