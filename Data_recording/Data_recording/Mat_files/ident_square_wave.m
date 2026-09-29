% ident_square_wave.m
% H. Garnier 
% 3/02/2025

% In other cases, the effects of nonlinearities are small. 
% In such a case, a linear model can provide a good approximation, 
% such that the nonlinear deviations are treated as disturbances. 
% Such approximations depend heavily on the input profile,
% the amplitude and frequency content of the excitation signal.
close all

clear;
fontsize=12;

load aero_responses_from_square_waves_7V_7V
%load aero_responses_from_square_waves_6V_12V
%load aero_responses_from_square_waves_6V_10V
%load aero_responses_from_square_waves_5V_12V
step_rsp=data_AERO;

Ts=0.002;

t=step_rsp(1,:)';
u_p=step_rsp(2,:)';
u_y=step_rsp(3,:)';
theta_p=step_rsp(4,:)';
theta_y=step_rsp(5,:)';
theta_p_dot=step_rsp(6,:)';
theta_y_dot=step_rsp(7,:)';


% Tracé des réponses en position et vitesse
figure
subplot(3,1,1)
plot(t,theta_p,'b','LineWidth',2),grid
legend('Pitch angle (°)')
set(gca,'FontSize',fontsize)
set(findall(gcf,'type','text'),'FontSize',fontsize)
subplot(3,1,2)
plot(t,theta_y,'r','LineWidth',2),grid
legend('Yaw angle (°)')
set(gca,'FontSize',fontsize)
set(findall(gcf,'type','text'),'FontSize',fontsize)
% subplot(5,1,3)
% plot(t,theta_p_dot,'b','LineWidth',2),grid
% legend('Pitch angular velocity ')
% set(gca,'FontSize',fontsize)
% set(findall(gcf,'type','text'),'FontSize',fontsize)
% subplot(5,1,4)
% plot(t,theta_y_dot,'r','LineWidth',2),grid
% legend('Yaw angular velocity')
% set(gca,'FontSize',fontsize)
% set(findall(gcf,'type','text'),'FontSize',fontsize)
subplot(3,1,3)
plot(t,u_p,t,u_y,'LineWidth',2),grid
legend('Pitch motor voltage', 'Yaw motor voltage')
xlabel('Time (sec)')
set(gca,'FontSize',fontsize)
set(findall(gcf,'type','text'),'FontSize',fontsize)

% % Identification d'un modèle d'état
t0=35;
n0=t0/Ts;
tf=140;
%nf=tf/Ts; 
nf=length(t);
theta_yd1=detrend(theta_y(n0:nf),1);
theta_pd0=detrend(theta_p(n0:nf),0);
u_p=u_p(n0:nf);
u_y=u_y(n0:nf);

data=iddata([theta_pd0-theta_pd0(1,1) theta_yd1-theta_yd1(1,1)],[u_p u_y],Ts);

Mss=ssest(data,4,'Form','canonical');
figure
opt = compareOptions('InitialCondition','zero');
%[theta_p_sim,fit_theta_p]=compare(data_pitch,Gpv_p,Gtheta_Vp,Inf,opt);
compare(data,Mss,Inf,opt);
grid
set(gca,'FontSize',fontsize)
set(findall(gcf,'type','text'),'FontSize',fontsize)
% 
%save Aero_model4_HG Mss_SW200s


% % Identification fonctions de transfert SIMO - pitch
%  data=iddata([theta_p],[u_p u_y],Ts);
% M_p=tfest(data,[2 2],[0 0],[0 0]); %Résultats moins bons qu'avec tfsrivc ! 80% FIT au lieu de 81,7%
% %M=ssest(data,3,'Form','canonical');
% figure
% opt = compareOptions('InitialCondition','zero');
% %[theta_p_sim,fit_theta_p]=compare(data_pitch,Gpv_p,Gtheta_Vp,Inf,opt);
% compare(data,M_p,Inf,opt);
% grid
% set(gca,'FontSize',fontsize)
% set(findall(gcf,'type','text'),'FontSize',fontsize)
% % 
% %
% % Identification fonctions de transfert SIMO - yaw
%  data=iddata([theta_y_dot],[u_p u_y],Ts);
% M_y=tfest(data,[2 1],[0 0],[0 0]); %Résultats moins bons qu'avec tfsrivc ! 80% FIT au lieu de 81,7%
% %M=ssest(data,3,'Form','canonical');
% figure
% opt = compareOptions('InitialCondition','zero');
% %[theta_p_sim,fit_theta_p]=compare(data_pitch,Gpv_p,Gtheta_Vp,Inf,opt);
% compare(data,M_y,Inf,opt);
% grid
% set(gca,'FontSize',fontsize)
% set(findall(gcf,'type','text'),'FontSize',fontsize)