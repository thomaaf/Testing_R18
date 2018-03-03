%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% Setup Analysis ----------------------------------------------------------
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
close all

%Analysis priority*********************
% 0=Everything 1=Initial 2=Roll&Pitch 3=Springs&Dampers 4=Response 5=Tires 
% 6=Aerodynamics 7=g-g
Analysis = [0,10, 11];
Loadcase = 0;
%**************************************
%% Load Data
NewSim = menu('                Load latest simulation?                ','Yes','No');
if NewSim == 1
    %cd('C:\CM_Projects\VehicleDynamics_2017\src_cm4sl\Mat_files');
    cd('E:\CM_Projects\R18_TV\src_cm4sl\Mat_files');
    load('VD_Vhcl&Driver.mat');
    load('VD_TireData.mat');
    load('VD_Aerodynamics.mat');
    %load('VD_Susp.mat');
    load('eng_torque.mat');
    cd('E:\CM_Projects\R18_TV\src_cm4sl');
else
%     cd('D:\OneDrive\Dokumenter\Revolve2017\Hjuloppheng\CarAnalysisMatLab\SavedSimulationData\');
%     [file, path] = uigetfile('*.mat', 'Please specify file to load');
%     cd(path);
%     load(file);
%     cd('D:\OneDrive\Dokumenter\Revolve2017\Hjuloppheng\CarAnalysisMatLab');
end

SteeringAngleFL = [-23.80 -22.65 -21.50 -20.34 -19.18 -18.03 -16.86 -15.70 -14.53 -13.35 -12.18 -10.99 -9.80 -8.61 -7.40 -6.19 -4.97 -3.75 -2.51 -1.26 0 1.27 2.56 3.85 5.17 6.50 7.84 9.21 10.59 12.00 13.42 14.88 16.36 17.87 19.41 20.99 22.61 24.26 25.97 27.73 29.54];
SteeringAngleFR = [-29.54 -27.73 -25.97 -24.26 -22.61 -20.99 -19.41 -17.87 -16.36 -14.88 -13.42 -12.00 -10.59 -9.21 -7.84 -6.50 -5.17 -3.85 -2.56 -1.27 0 1.26 2.51 3.75 4.97 6.19 7.40 8.61 9.80 10.99 12.18 13.35 14.53 15.70 16.86 18.03 19.18 20.34 21.50 22.65 23.80];


time = Vhcl.Velocity.Vx.Time;
%% Time window
dt = round(time(end)/length(time),3);
T_start = 0;
T_end = time(end);
t = T_start/dt+1:T_end/dt;

% T_start2 = 33;
% T_end2 = 34;
% t = [t T_start2/dt+1:T_end2/dt];


%% Initial Analysis
if sum(ismember([0,1],Analysis))
    figure(100)
    % Speed
    subplot(6,1,1)
    plot(time(t),Vhcl.Velocity.Vx.data(t),'b');grid on;hold on
    legend('Speed [m/s]','Location','NorthWest');title(['Initial Analysis - Laptime: ' num2str(max(Vhcl.TriggerPointTime.data)) 's'])
    % Throttle & Brake
    subplot(6,1,2)
    plot(time(t),Vhcl.Driver.Throttle.data(t),'g');grid on;hold on
    plot(time(t),Vhcl.Driver.Brake.data(t),'r');
    legend('Trottle','Brake','Location','NorthWest')
    % Power
    subplot(6,1,3)
    plot(time(t),Vhcl.Power.data(t)./1000);grid on;hold on
    legend('Power [kW]','Location','NorthWest');ylim([-85 85]);
    % Steering
    subplot(6,1,4)
    plot(time(t),Vhcl.Driver.SteerAng.data(t),'b');grid on;hold on
    legend('Steering wheel angle [deg]','Location','NorthWest')
    %Oversteer
    subplot(6,1,5)
    %plot(time(t),Vhcl.OM.steadystate.data(t).*(180/pi),'r');hold on;
    %plot(time(t),Vhcl.OM.transient.data(t),'m');set(gca,'xticklabel',[]);
    grid on;legend('OM Steady State','Location','NorthWest')
    ylim([-2 2])
    %Lateral acceleration
    subplot(6,1,6)
    plot(time(t),Vhcl.Acceleration.Ay.data(t),'r');grid on;hold on
    legend('Ay [m/s^2]','Location','NorthWest');xlabel('Time [s]')
end
%% Roll and pitch
if sum(ismember([0,2],Analysis))
    %Roll
    figure(200)
    subplot(4,1,1)
    plot(time(t),Vhcl.Roll.Angle.data(t));grid on;hold on;title('Roll angle');
    %legend('Roll angle [deg]','Location','NorthWest');title(['Roll motion - Laptime: ' num2str(max(Vhcl.TriggerPointTime.data)) 's']);
    subplot(4,1,2)
    plot(time(t),Vhcl.Roll.Rate.data(t));grid on;hold on
    legend('Roll rate [deg/s]','Location','NorthWest');title('Roll rate');
    subplot(4,1,3)
    plot(time(t),Vhcl.Roll.Acceleration.data(t)); grid on;hold on
    legend('Roll acceleration [deg/s^2]','Location','NorthWest');title('Roll acc');
    subplot(4,1,4)
    plot(time(t),Vhcl.Acceleration.Ay.data(t));grid on;hold on
    legend('Ay [m/s^2]','Location','NorthWest');xlabel('Time [s]');title('');
    %Pitch
    figure(300)
    subplot(4,1,1)
    plot(time(t),Vhcl.Pitch.Angle.data(t),'r');grid on; line([0 9],[-0.04888 -0.04888]);
    legend('Pitch angle [deg]','Location','NorthWest');title('Pitch motion');
    subplot(4,1,2)
    plot(time(t),Vhcl.Pitch.Rate.data(t));grid on;hold on
    legend('Pitch rate [deg/s]','Location','NorthWest');title('Pitch rate');
    subplot(4,1,3)
    plot(time(t),Vhcl.Pitch.Acceleration.data(t)); grid on;hold on
    legend('Pitch acceleration [deg/s^2]','Location','NorthWest');title('Pitch acc');
    subplot(4,1,4)
    plot(time(t),Vhcl.Acceleration.Ax.data(t)); grid on;hold on
    legend('Longitudinal acceleration [m/s^2]','Location','NorthWest');title('Ax');
    xlabel('Time [s]');
end

%% Tires
if sum(ismember([5,0],Analysis)) %Analysis==5 || Analysis==0
    figure(400)
    % Throttle & Brake
    subplot(2,2,2)
    plot(time(t),Vhcl.Driver.Throttle.data(t),'g');grid on;hold on
    plot(time(t),Vhcl.Driver.Brake.data(t),'r');%set(gca,'xticklabel',[])
    legend('Trottle','Brake','Location','NorthWest');
    % Steering
    subplot(2,2,1)
    plot(time(t),Vhcl.Driver.SteerAng.data(t),'b');grid on;hold on
    legend('Steering wheel angle [deg]','Location','NorthWest')
    %title(['                                                      Tire Analysis - Laptime: ' num2str(max(Vhcl.TriggerPointTime.data)) 's']);
    % Slip Angle
    subplot(2,2,3)
    plot(time(t),Tire.SlipAngle.FL.data(t));hold on;grid on;
    plot(time(t),Tire.SlipAngle.FR.data(t));%set(gca,'xticklabel',[])
    plot(time(t),Tire.SlipAngle.RL.data(t));
    plot(time(t),Tire.SlipAngle.RR.data(t));
    legend('SA-FL','SA-FR','SA-RL','SA-RR','Location','NorthWest')
    subplot(2,2,4);
    plot(time(t),Tire.SlipRatio.FL.data(t));hold on;grid on;
    plot(time(t),Tire.SlipRatio.FR.data(t));%set(gca,'xticklabel',[])
    plot(time(t),Tire.SlipRatio.RL.data(t));
    plot(time(t),Tire.SlipRatio.RR.data(t));ylim([-0.25 0.25]);
    legend('SR-FL','SR-FR','SR-RL','SR-RR','Location','NorthWest')
    
    figure(401)
    % Fx
    subplot(2,2,1)
    plot(time(t),Tire.Fx.FL.data(t));hold on;grid on;
    plot(time(t),Tire.Fx.FR.data(t));%set(gca,'xticklabel',[])
    plot(time(t),Tire.Fx.RL.data(t));
    plot(time(t),Tire.Fx.RR.data(t));
    legend('Fx-FL','Fx-FR','Fx-RL','Fx-RR','Location','NorthWest')
    % Fy
    subplot(2,2,2)
    plot(time(t),Tire.Fy.FL.data(t));hold on;grid on;
    plot(time(t),Tire.Fy.FR.data(t));%set(gca,'xticklabel',[])
    plot(time(t),Tire.Fy.RL.data(t));
    plot(time(t),Tire.Fy.RR.data(t));xlabel('Time [s]')
    legend('Fy-FL','Fy-FR','Fy-RL','Fy-RR','Location','NorthWest')
    % Fy
    subplot(2,2,3)
    plot(time(t),Tire.Fz.FL.data(t));hold on;grid on;
    plot(time(t),Tire.Fz.FR.data(t));
    plot(time(t),Tire.Fz.RL.data(t));
    plot(time(t),Tire.Fz.RR.data(t));xlabel('Time [s]');
    legend('Fz-FL','Fz-FR','Fz-RL','Fz-RR','Location','NorthWest');
    % Mz
    subplot(2,2,4)
    plot(time(t),Tire.Mz.FL.data(t));hold on;grid on;
    plot(time(t),Tire.Mz.FR.data(t));xlabel('Time [s]')
    legend('Mz-FL','Mz-FR','Location','NorthWest')
    
    print_step = 1;
    figure()
    subplot(3,1,1)
    plot(Tire.SlipAngle.FL.data(t(1):print_step:t(end)),Tire.Fy.FL.data(t(1):print_step:t(end)),'.');grid on;hold on;
    plot(Tire.SlipAngle.FR.data(t(1):print_step:t(end)),Tire.Fy.FR.data(t(1):print_step:t(end)),'.');
    plot(Tire.SlipAngle.RL.data(t(1):print_step:t(end)),Tire.Fy.RL.data(t(1):print_step:t(end)),'.');
    plot(Tire.SlipAngle.RR.data(t(1):print_step:t(end)),Tire.Fy.RR.data(t(1):print_step:t(end)),'.');
    title('Fy vs. SA');xlabel('Slip angle [deg]');ylabel('Lateral force [N]')
    subplot(3,1,2)
    plot(Tire.SlipRatio.FL.data(t(1):print_step:t(end)),Tire.Fx.FL.data(t(1):print_step:t(end)),'.');grid on;hold on;
    plot(Tire.SlipRatio.FR.data(t(1):print_step:t(end)),Tire.Fx.FR.data(t(1):print_step:t(end)),'.');
    plot(Tire.SlipRatio.RL.data(t(1):print_step:t(end)),Tire.Fx.RL.data(t(1):print_step:t(end)),'.');
    plot(Tire.SlipRatio.RR.data(t(1):print_step:t(end)),Tire.Fx.RR.data(t(1):print_step:t(end)),'.');
    title('Fx vs. SR');xlabel('Slip ratio [-]');ylabel('Longitudinal force [N]')
    subplot(3,1,3)
    plot(Tire.SlipAngle.FL.data(t(1):print_step:t(end)),Tire.Mz.FL.data(t(1):print_step:t(end)),'.');grid on;hold on;
    plot(Tire.SlipAngle.FR.data(t(1):print_step:t(end)),Tire.Mz.FR.data(t(1):print_step:t(end)),'.');
    title('Mz vs. SA');xlabel('Slip angle [deg]');ylabel('Alignment torque [Nm]')
end
if Loadcase
    print_step = 10;
    figure()
    %Lateral
    subplot(4,1,1)
    plot(Vhcl.Speed.data(t(1):print_step:t(end)),abs(Tire.Fy.FL.data(t(1):print_step:t(end))),'.');grid on;hold on
    plot(Vhcl.Speed.data(t(1):print_step:t(end)),abs(Tire.Fy.FR.data(t(1):print_step:t(end))),'.');title('Fy');
    plot(Vhcl.Speed.data(t(1):print_step:t(end)),abs(Tire.Fy.RL.data(t(1):print_step:t(end))),'.');xlabel('Speed [m/s]')
    plot(Vhcl.Speed.data(t(1):print_step:t(end)),abs(Tire.Fy.RR.data(t(1):print_step:t(end))),'.');ylabel('Force [N]')
    %Longitudinal 
    subplot(4,1,2)
    plot(Vhcl.Speed.data(t(1):print_step:t(end)),abs(Tire.Fx.FL.data(t(1):print_step:t(end))),'.');grid on;hold on
    plot(Vhcl.Speed.data(t(1):print_step:t(end)),abs(Tire.Fx.FR.data(t(1):print_step:t(end))),'.');title('Fx');
    plot(Vhcl.Speed.data(t(1):print_step:t(end)),abs(Tire.Fx.RL.data(t(1):print_step:t(end))),'.');xlabel('Speed [m/s]')
    plot(Vhcl.Speed.data(t(1):print_step:t(end)),abs(Tire.Fx.RR.data(t(1):print_step:t(end))),'.');ylabel('Force [N]') 
    %Longitudinal brake
    %Normal load
    subplot(4,1,4)
    plot(Vhcl.Speed.data(t(1):print_step:t(end)),abs(Tire.Fz.FL.data(t(1):print_step:t(end))),'.');grid on;hold on
    plot(Vhcl.Speed.data(t(1):print_step:t(end)),abs(Tire.Fz.FR.data(t(1):print_step:t(end))),'.');title('Fz');
    plot(Vhcl.Speed.data(t(1):print_step:t(end)),abs(Tire.Fz.RL.data(t(1):print_step:t(end))),'.');xlabel('Speed [m/s]')
    plot(Vhcl.Speed.data(t(1):print_step:t(end)),abs(Tire.Fz.RR.data(t(1):print_step:t(end))),'.');ylabel('Force [N]')     
end
    
%% Aerodynamics
if sum(ismember([0,6],Analysis))
    figure(500)
    %Forces
    subplot(4,1,1)
    plot(time(t),-Aero.Force.Drag.data(t));hold on;grid on;
    plot(time(t),Aero.Force.Lateral.data(t));%set(gca,'xticklabel',[])
    plot(time(t),-Aero.Force.Lift.data(t));title(['Aerodynamics - Laptime: ' num2str(max(Vhcl.TriggerPointTime.data)) 's']);
    legend('Drag [N]','Lat [N]','Downforce [N]','Location','NorthWest');
    %Moments
    subplot(4,1,2)
    plot(time(t),Aero.Moment.Roll.data(t));hold on;grid on;
    plot(time(t),Aero.Moment.Pitch.data(t));%set(gca,'xticklabel',[])
    plot(time(t),Aero.Moment.Yaw.data(t));
    legend('Roll Moment [Nm]','Pitch Moment [Nm]','Yaw Moment [Nm]','Location','NorthWest');
    %DRS
    subplot(4,1,3)
    %plot(time(t),Aero.DRS.data(t));grid on;hold on
    %legend('DRS open/closed','Location','NorthWest');
    %MagicMumber
    subplot(4,1,4)
    plot(time(t),Aero.MagicMumber.data(t));grid on;hold on
    legend('MagicMumber','Location','NorthWest');ylim([0.4 0.6])
end
%% "g-g" and Friction ellipse
print_step = 100;
if sum(ismember([0,7],Analysis))
    figure(600)
    % g-g
    subplot(2,1,1)
    plot(Vhcl.Acceleration.Ay.data(t(1):print_step:t(end))./9.81,Vhcl.Acceleration.Ax.data(t(1):print_step:t(end))./9.81,'.');grid on;hold on
    xlabel('Lateral acceleration [g]');ylabel('Longitudinal acceleration [g]');title(['g-g - Laptime: ' num2str(max(Vhcl.TriggerPointTime.data)) 's']);
    ylim([-3 3]);xlim([-3 3]);
    % Friction ellipse
    TotalFx = Tire.Fx.FL.data+Tire.Fx.FR.data+Tire.Fx.RL.data+Tire.Fx.RR.data;
    TotalFy = Tire.Fy.FL.data+Tire.Fy.FR.data+Tire.Fy.RL.data+Tire.Fy.RR.data;
    subplot(2,1,2)
    plot(TotalFy(t(1):print_step:t(end)),TotalFx(t(1):print_step:t(end)),'.');grid on;hold on
    xlabel('Lateral force [N]');ylabel('Longitudinal force [N]');title(['Fx-Fy - Laptime: ' num2str(max(Vhcl.TriggerPointTime.data)) 's']);
end
%% Steady state response
if sum(ismember([0,8],Analysis))
    figure(700)
    subplot(3,1,1)
    plot(Vhcl.Acceleration.Ay.data(t),Vhcl.Driver.SteerAng.data(t),'.');grid on;xlabel('Lateral Acceleration [m/s^2]');ylabel('Steering Wheel Angle [deg]');hold on
    subplot(3,1,2)
    plot(Vhcl.Acceleration.Ay.data(t),Vhcl.Roll.Angle.data(t),'.');grid on;xlabel('Lateral Acceleration [m/s^2]');ylabel('Chassis Roll Angle [deg]');hold on
    subplot(3,1,3)
    plot(Vhcl.Acceleration.Ay.data(t),Vhcl.Driver.SteerTrq.data(t),'.');grid on;xlabel('Lateral Acceleration [m/s^2]');ylabel('Steering Wheel Torque [Nm]');hold on
    
    figure(800)
    subplot(2,2,1)
    plot(Vhcl.Acceleration.Ay.data(t),Tire.SlipAngle.FL.data(t),'.');grid on;xlabel('Lateral Acceleration [m/s^2]');ylabel('Slip Angle [deg]');legend('FL');hold on
    subplot(2,2,2)
    plot(Vhcl.Acceleration.Ay.data(t),Tire.SlipAngle.FR.data(t),'.');grid on;xlabel('Lateral Acceleration [m/s^2]');ylabel('Slip Angle [deg]');legend('FR');hold on
    subplot(2,2,3)
    plot(Vhcl.Acceleration.Ay.data(t),Tire.SlipAngle.RL.data(t),'.');grid on;xlabel('Lateral Acceleration [m/s^2]');ylabel('Slip Angle [deg]');legend('RL');hold on
    subplot(2,2,4)
    plot(Vhcl.Acceleration.Ay.data(t),Tire.SlipAngle.RR.data(t),'.');grid on;xlabel('Lateral Acceleration [m/s^2]');ylabel('Slip Angle [deg]');legend('RR');hold on
end
%% Step response
if sum(ismember([0,9],Analysis))
    figure(900)
    yyaxis left;plot(time(t),Vhcl.Driver.SteerAng.data(t),'.');hold on;grid on;ylabel('Steering-Wheel Angle [deg]')
    yyaxis right;plot(time(t),Vhcl.Acceleration.Ay.data(t),'linewidth',1);grid on;ylabel('Lateral acceleration [deg]');xlabel('Time [s]');hold on
    
    figure(1000)
    yyaxis left;plot(time(t),Vhcl.Driver.SteerAng.data(t),'.');hold on;grid on;ylabel('Steering-Wheel Angle [deg]')
    yyaxis right;plot(time(t),Vhcl.YawRate.data(t),'linewidth',1);grid on;ylabel('Yaw Rate [rad/s]');xlabel('Time [s]');hold on
end

%% Yaw moment
if ismember(10,Analysis)
    Izz = 51.12;
    beta = atand(Vhcl.Velocity.Vy.data(t)./Vhcl.Velocity.Vx.data(t));
    yaw_moment = Izz*Vhcl.Yaw.Acceleration.data(t);
    figure(1100)
    scatter3(Vhcl.Acceleration.Ay.data(t), yaw_moment, beta)
    xlabel('Lat acc [m/s^2]');ylabel('Yaw moment');
    
    figure(1101)
    steeredAngFL = interp1(linspace(-105,105,41),SteeringAngleFL,Vhcl.Driver.SteerAng.data(t));
    scatter(steeredAngFL, Vhcl.Yaw.Rate.data(t))
    xlabel('\delta FL');ylabel('Yaw Rate [rad/s]');grid on;
    %if beta

end
% Warp
if ismember(11, Analysis)
    figure(1200)
    warp = (Tire.Fz.FR.data(t) + Tire.Fz.RL.data(t)) - (Tire.Fz.FL.data(t) + Tire.Fz.RR.data(t));
    scatter(Vhcl.Acceleration.Ay.data(t), warp);
    xlabel('Lat acc [m/s^2]');ylabel('Warp Force per Wheel [N]'); hold on;
    
end


%% Save file
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
if  NewSim == 1
    SaveFile = menu('            Save simulation results to .mat file?              ','  Yes  ','  No  ');
    if SaveFile==1
        UserInput = inputdlg({'Filename','Folder name (Acceleration,Autocross,SinusSweep,Skidpad,SteadyStateCorner,StepSteer)'},'Save to file',[1 75;1 75]);
        if isempty(UserInput)
            disp('***Save canceled!***')
            disp('New result file was not generated');
        else
            filename = ['D:\OneDrive\Dokumenter\Revolve2017\Hjuloppheng\CarAnalysisMatLab\SavedSimulationData\',strjoin(UserInput(2)),'\',strjoin(UserInput(1)),'.mat'];
            save(filename, 'Vhcl', 'Tire', 'Aero','torques')
        end
    else
        disp('New result file was not generated');
    end
else
    RunNewSim = menu('                     Run new simulation?               ','  Yes  ','  No  ');
    if RunNewSim==1
        cd('C:\CM_Projects\VehicleDynamics_2017\src_cm4sl');
    end
end