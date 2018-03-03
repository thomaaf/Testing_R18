% Parameter file for Revolve TV 2016 -> 2017
% Initialize parameters automatically when the model gets loaded:
% Pushed 04.02.18
load('Buses_2017.mat');

% STRUCTS
yawRateControl = struct;
yawMomentGenerator = struct;
tractionControl = struct;
powerLimitControl = struct;
energyLimitControl = struct;
settings = struct;
car_params = struct;
matlab_params = struct;
motor_data = struct;
%speedEstimation = struct;

% ------------------------------ 
% "SETTINGS"
% ------------------------------
settings.KERS_active = uint32(1);           % 1 = Kers can be used, 0 = Kers is not used
settings.select_Fz_est_method = uint32(2);  % 1 = LoadTransfer, 2 = DamperBased            not in use
settings.select_velocity_vx = uint32(2);    % 1 = estimated, 2 = INS, 3 = Optical-N/A      not in use
settings.select_velocity_vy = uint32(2);    %                2 = INS, 3 = Optical-N/A      not in use
% 1 = QP , 2 = Full allocation, 
% 3 = static 25  , 4 = with negative, does not use Fz for alpha
settings.TV_Method = uint32(1); 
settings.use_estimated_Fz = uint32(1); % 1 = yes, 0 = no  (affects sim only

settings.max_motor_torque = single(21);
settings.max_motor_brake_torque = single(18);
% The speed setpoint when driving
settings.max_RPM = single(19500);
settings.RPM_scaling = single(1);    % to be implemented at later point
% The speed at which KERS will be allowed again
settings.KERS_speed_limit_hyst_high = single(3); % Can not KERS under 5[Km/h] -> 5/3.6  = 1.3889
% The speed at which KERS will be turned off
settings.KERS_speed_limit_hyst_low = single(1.0);
settings.KERS_motor_RPM_setpoint = single(1200);
settings.mu = single(1.4); % 2.5
settings.max_battery_discharge =single(160); % Ampere 180|190 (Melasta single cell cont-discharge 65A/ peak 98A

i_Diff = 16;   % ? ----------------------------- ???  15.5

% ------------------------------
% Car Parameters
% ------------------------------

car_params.GR = single(15.5);
car_params.t_f = single(0.6); % CM_WC=0.6  Half trackwidth front
car_params.t_r = single(0.590); %CM_WC= 590  gnist=0.585 Half trackwidth rear
car_params.l = single(1.53); % Wheelbase   1.53    other values to tune ss turn characteristic
car_params.l_f = single(0.826); % 0.813 Distance CoG to front axle
car_params.l_r = single(0.704); %0.717% Distance CoG to rear axle
car_params.CoG_height = single(0.276); % Height of Center of gravity [m]
car_params.InitRideheight = 0.045;      % Initial rideheight [m]

car_params.spring_constant_front            = single(35.025);  
car_params.spring_constant_rear             = single(39.404);
%make damper values imported exptern

car_params.damper_pull_push_values_front =  single([-331    -190   0   84    147]);
car_params.damper_pull_push_values_rear =   single([-333    -209   0   93    148]);
car_params.damper_table_breakpoint =        single([-0.250 -0.03   0   0.03  0.250]*1000);

car_params.arb_f_Nm_rad =  10000;% 6800; 5060; %ARB stiffness Nm/rad
car_params.arb_r_Nm_rad =  10000;% 7000; 5060; %ARB stiffness Nm/rad

car_params.motion_ratio_front               = single(1);
car_params.motion_ratio_rear                = single(1);
car_params.preload_front_mm                 = single(0); % not acounted for by 21.01.18
car_params.preload_rear_mm                  = single(0);

car_params.unsprung_mass_per_wheel_newton_front   = single((5.73+7.23)*9.81); 
car_params.unsprung_mass_per_wheel_newton_rear    = single((5.42+7.13)*9.81); 

car_params.driver_mass = single(75);
car_params.car_mass    = single(185);
car_params.r_eff       = single(0.230); %     Continental  0.235     // FS 205   0.250




%Motor / inverter parameters -- for more see end of file (details for
%simulation) //temp lmits commented out due to HW NaN error- not time to fix 2017
car_params.AMK_motor_upper_max_temp         = single(90);
car_params.AMK_motor_cutoff_temp            = single(120);
car_params.AMK_inverter_upper_max_temp      = single(90);
car_params.AMK_inverter_cutoff_temp         = single(120);

car_params.Kt   = single(0.26);         % Torque constant
car_params.Lq   = single(0.00054);      % Quadrature axis inductance [Henry] 
car_params.Ld   = single(0.00044);      % Direct access inductance [Henry]
car_params.Jw   = single(0.78);         % Total inertia the motor sees
car_params.Mn   = single(0.0098);       % Motor nominal torque(9.8Nm), for calculating setpoint in [0.1]% of Mn (see 8.2.4 in inverter DS)8.1.3


% Aero sim values by J.m. Haaland
% car_params.aero_lift_coeff = single(3.163);
% car_params.aero_ref_area   = single(1.1008);
car_params.AirDens = single(1.201);
car_params.aero_lift_coeff = single(3.295);     %For more detailed coeff. see further down
car_params.aero_ref_area   = single(1.166);
car_params.DRSenable = uint32(1);               % 0 No DRS (closed), 1 DRS active, 2 DRS open
car_params.DRS_Pmin = single(50000);            % Minimum power output for DRS to open        
car_params.DRS_SteerMax = single(7);            % Maximum steering angle for DRS to open
car_params.AeroRefX = 1.21672;                  % Distance from FR1 to aerodynamic load application point
%car_params.aeroSetting = uint32(1);


% ------------------------------
% Yaw Rate Control Parameters (method(2))
% ------------------------------
yawRateControl.Ku = single(0); % Understeer coefficient
yawRateControl.r_ref_tuning_param = single(1.59); %1.5 -> car_params.l 
yawRateControl.negative_torque_limit = 15;
yawRateControl.max_moment = single(2000); %1000

yawRateControl.Kp_start = single(1000); %      450 || 4500 || 1000 skid
yawRateControl.Kp_end = single(500);
yawRateControl.Kp_scaling = single(1); 

yawRateControl.Ki_start    = single(0); % 2000 
yawRateControl.Ki_end      = single(0);
yawRateControl.Ki_scaling  =    single(1);
yawRateControl.Ksat        =   single(0); % 25 this is the scaling of the integral backstepping

yawRateControl.Kd_start = single(0); % 50
yawRateControl.Kd_end   = single(0); % 50
yawRateControl.Kd_scaling = single(1); %1

yawRateControl.lookup_speed_end_kmh = single(30); % 50

yawRateControl.ref_weight  = single(1);
yawRateControl.meas_weight = single(0.5); 

yawRateControl.enable_r_ref_limit = uint32(0);  % 1
yawRateControl.mu_scaling_r_ref_limit = single(1.5);

yawRateControl.enable = uint32(1);


%1 for Yaw Rate Generator (P-(D))
%2 for Yaw Rate Control (PID)
yawRateControl.method = uint32(1);

%-------------------------------
% Yaw Rate Generator Parameters (method 1)
%-------------------------------

yawMomentGenerator.max_moment = single(2000);

%Proportinal term gains and brakepoint speeds.
yawMomentGenerator.Kp_gains  = single([2000 1500 1300 700 500]);
yawMomentGenerator.Kp_speeds = single([0 20 40 70 120]);

%Derivative term gains and brakepoint speeds (remember that this is 
%opposite of a traditional one) and introduces response, and reduces
%stability.
yawMomentGenerator.Kd_gains  = single([0 0 0 0 0 0]);
yawMomentGenerator.Kd_speeds = single([0 15 25 40 60 120]);


% ------------------------------
% Traction control Parameters
% ------------------------------

tractionControl.slip_ang_limit = single(0.2);
tractionControl.Slip_ratio_ref = single(0.15);
% Gains scheduled av torque request i tillegg ? 
% 
tractionControl.Kb = single(5); % 5 % Anti wind up gain

% While v_x is lower than full gain limit, Kp and Ki are multiplied 
% with a number between 0-1 which depends on how much torque the driver
% is requesting. 
tractionControl.full_gain_limit_kmh = single(10);

tractionControl.Kp_start = single(8);   % 7
tractionControl.Kp_end = single(150);   % 150
tractionControl.Kp_scaling = single(1); % OOOOBSBSS

tractionControl.Ki_start = single(120); % 120
tractionControl.Ki_end = single(450);   % 250
tractionControl.Ki_scaling = single(1);


tractionControl.Kp_braking = single(150); % 150
tractionControl.Ki_braking = single(450); % 450
tractionControl.lookup_speed_end_kmh = single(50);
tractionControl.enable = uint32(0);

% SLIP DETECTION
% settings for the slip detection traction control (This is prior to the optimization).
tractionControl.enable2 = uint32(1); % 1 for enable , 0 for disable

tractionControl.startValueSlip = single(0.75); %scales the torque from 1 to startValueSlip at slip detection.
tractionControl.startValueLock = single(1); %scales the torque from 1 to startValueLock at lock detection.
tractionControl.riseTime  = single(1200); %iterations from 0 to 1;

%sensitivity parameters for the slip detection
tractionControl.slip_rate_pos_lim = single(0.25);     %limit for slip, that corresponds to wheelslip. Can be moved up and down to make traction control more agressive.
tractionControl.slip_rate_neg_lim = single(-0.30);    % limit of negative slip that corresponds to wheels "locking", can be moved up and down to make traction control more/less agressive.
tractionControl.omega_der_slip_lim = single(2.5 *(9.81/0.225));  %Upper limit for plausible wheel angular acceleration, without wheelspin.
tractionControl.omega_der_lock_lim = single(-4.5*(9.81/0.225));  %lower limit for plausible wheel angular acceleration, without "wheellock".


% ------------------------------
% Power Limit Control Parameters
% ------------------------------
powerLimitControl.positive_power_limit = single(80000); % Positive effect limit
powerLimitControl.negative_power_limit = single(80000); %
powerLimitControl.Kp = single(0.000007);%(0.00005); % (0.00005  // 0.000007   //only 5 decimals in analyze :/
powerLimitControl.Ki = single(0.0007);  % single(0.0002) / 0.001  / .0015

powerLimitControl.p_positive = single(10); 
powerLimitControl.p_negative = single(11);  

powerLimitControl.upper_sat_limit = single(0);
powerLimitControl.lower_sat_limit = single(-2); % Based on Max effect = 120KW, 8/12 = 0.6
powerLimitControl.enable = uint32(1); %2 == PWL 2018 1 == enabled , 0 = disabled


% % ------------------------------
% % SPEED ESTIMATION
% % ------------------------------
% %Limits (Speed Estimation)
% speedEstimation.variance_limit = single(0.05);
% speedEstimation.RPM_derivative_limit = single(10000);
% speedEstimation.velocity_deviation_limit = single(3);
% % Gains
% speedEstimation.High_gain = single(2);
% speedEstimation.Low_gain = single(0);

% Struct for parameters only used in Matlab, not in generated control
% system code.
Use_Joystick = 1; % 1 = No, -1 = yes
battery_voltage = single(530);
matlab_params.sensor_sampling_time = 0.01;
matlab_params.control_system_sample_time = 0.01;
matlab_params.sample_time = 0.001;

% Controller settings for motors inside Carmaker blocks
% ------------------------------
Kpd = 1.72; % V/A
%Kpq = 1.9; % V/A Torque Controller
Kpq = 1; % 1.9 V/A Torque Controller
Tnq = 10; % 5 ms  Time constant Torque controller
q_BCC = 10; %Torque controller
Tnd = 1.2; % ms Time constant
Kpq2 = 20; % % adaption gain
%INVERTER SPEED
Kp = 1; % A/V Voltage controller gain
Tn_n = 0; % 1 ms. Time constant Speed
SpdC_BCC = 0; % 25
Tn = 6; % ms Voltage time constant
Speed_Pos_sat = 21;%(21/Mn)*1000;
Speed_Neg_sat = -21;%-(21/Mn)*1000;

%Name the structures for Embedded Coder
% SETTINGS
Simulink_Settings = Simulink.Parameter;
Simulink_Settings.Value = settings;
Simulink_Settings.CoderInfo.StorageClass = 'ImportedExtern';
settings = Simulink_Settings;
clear Simulink_Settings;

busInfo=Simulink.Bus.createObject(settings.Value);
Settings = eval(busInfo.busName);
settings.DataType='Bus: Settings';
clear(busInfo.busName);
clear busInfo;

%*****************************%
% CAR_PARAMS
Simulink_Car_params = Simulink.Parameter;
Simulink_Car_params.Value = car_params;
Simulink_Car_params.CoderInfo.StorageClass = 'ImportedExtern';
car_params = Simulink_Car_params;
clear Simulink_Car_params;

busInfo=Simulink.Bus.createObject(car_params.Value);
CarParams = eval(busInfo.busName);
car_params.DataType='Bus: CarParams';
clear(busInfo.busName);
clear busInfo;


%*****************************%
% PowerLimitControl
Simulink_el = Simulink.Parameter;
Simulink_el.Value = powerLimitControl;
Simulink_el.CoderInfo.StorageClass = 'ImportedExtern';%'ExportedGlobal';
powerLimitControl = Simulink_el;
clear Simulink_el;

busInfo=Simulink.Bus.createObject(powerLimitControl.Value);
PowerLimitControl = eval(busInfo.busName);
powerLimitControl.DataType='Bus: PowerLimitControl';
clear(busInfo.busName);
clear busInfo;

%*****************************%
%traction control
Simulink_tc = Simulink.Parameter;
Simulink_tc.Value = tractionControl;
Simulink_tc.CoderInfo.StorageClass = 'ImportedExtern';%'ExportedGlobal';
tractionControl = Simulink_tc;
clear Simulink_tc;

busInfo=Simulink.Bus.createObject(tractionControl.Value);
TractionControl = eval(busInfo.busName);
tractionControl.DataType='Bus: TractionControl';
clear(busInfo.busName);
clear busInfo;

% %*****************************%
% %Speed Estimation
% Simulink_speedEst = Simulink.Parameter;
% Simulink_speedEst.Value = speedEstimation;
% Simulink_speedEst.CoderInfo.StorageClass = 'ExportedGlobal';
% speedEstimation = Simulink_speedEst;
% clear Simulink_speedEst;
% 
% busInfo=Simulink.Bus.createObject(speedEstimation.Value);
% SpeedEstimation = eval(busInfo.busName);
% speedEstimation.DataType='Bus: SpeedEstimation';
% clear(busInfo.busName);
% clear busInfo;

%*****************************%
%Yaw rate control struct
Simulink_yawRate = Simulink.Parameter;
Simulink_yawRate.Value = yawRateControl;
Simulink_yawRate.CoderInfo.StorageClass = 'ImportedExtern';%'ExportedGlobal';
yawRateControl = Simulink_yawRate;
clear Simulink_yawRate;

busInfo=Simulink.Bus.createObject(yawRateControl.Value);
YawRateControl = eval(busInfo.busName);
yawRateControl.DataType='Bus: YawRateControl';
clear(busInfo.busName);
clear busInfo;

%*****************************%
%Yaw rate generator struct
Simulink_yawRate2 = Simulink.Parameter;
Simulink_yawRate2.Value = yawMomentGenerator;
Simulink_yawRate2.CoderInfo.StorageClass = 'ImportedExtern';%'ExportedGlobal';
yawMomentGenerator = Simulink_yawRate2;
clear Simulink_yawRate2;

busInfo=Simulink.Bus.createObject(yawMomentGenerator.Value);
YawMomentGenerator = eval(busInfo.busName);
yawMomentGenerator.DataType='Bus: YawMomentGenerator';
clear(busInfo.busName);
clear busInfo;

%*****************************%
%EnergyLimitControl
Simulink_energyLimit = Simulink.Parameter;
Simulink_energyLimit.Value = energyLimitControl;
Simulink_energyLimit.CoderInfo.StorageClass =  'ImportedExtern';%'ExportedGlobal';
energyLimitControl = Simulink_energyLimit;
clear Simulink_yawRate;

busInfo=Simulink.Bus.createObject(energyLimitControl.Value);
EnergyLimitControl = eval(busInfo.busName);
energyLimitControl.DataType='Bus: EnergyLimitControl';
clear(busInfo.busName);
clear busInfo;




%Motor limits based on datasheet
motor_data.MaxTorqueValues = [21 21 15 0];
motor_data.MaxTorqueBreakpoints = [0 15000 19500 19999];
motor_data.eff = single(reshape([0.6437000036239624,0.5799999833106995,0.44999998807907104,0.3499999940395355,0.28999999165534973,0.23999999463558197,0.20000000298023224,0.17000000178813934,0.14000000059604645,0.12999999523162842,0.10999999940395355,0.7099999785423279,0.699999988079071,0.6100000143051148,0.5099999904632568,0.4399999976158142,0.3799999952316284,0.33000001311302185,0.28999999165534973,0.25,0.2199999988079071,0.20000000298023224,0.7300000190734863,0.7699999809265137,0.7300000190734863,0.6700000166893005,0.6100000143051148,0.550000011920929,0.5,0.44999998807907104,0.4000000059604645,0.36000001430511475,0.33000001311302185,0.7400000095367432,0.800000011920929,0.7900000214576721,0.7400000095367432,0.6899999976158142,0.6399999856948853,0.5899999737739563,0.5400000214576721,0.5,0.46000000834465027,0.41999998688697815,0.75,0.8199999928474426,0.8199999928474426,0.7799999713897705,0.7400000095367432,0.699999988079071,0.6600000262260437,0.6100000143051148,0.5699999928474426,0.5299999713897705,0.49000000953674316,0.7599999904632568,0.8399999737739563,0.8500000238418579,0.8299999833106995,0.800000011920929,0.7699999809265137,0.7400000095367432,0.699999988079071,0.6600000262260437,0.6200000047683716,0.5899999737739563,0.7699999809265137,0.8500000238418579,0.8799999952316284,0.8700000047683716,0.8600000143051148,0.8399999737739563,0.8100000023841858,0.7799999713897705,0.7599999904632568,0.7200000286102295,0.699999988079071,0.7699999809265137,0.8500000238418579,0.8899999856948853,0.8799999952316284,0.8700000047683716,0.8500000238418579,0.8299999833106995,0.8100000023841858,0.7799999713897705,0.7599999904632568,0.7300000190734863,0.7699999809265137,0.8600000143051148,0.8999999761581421,0.8999999761581421,0.8899999856948853,0.8700000047683716,0.8500000238418579,0.8299999833106995,0.8100000023841858,0.7900000214576721,0.6700000166893005,0.7799999713897705,0.8600000143051148,0.8999999761581421,0.9100000262260437,0.8999999761581421,0.8899999856948853,0.8700000047683716,0.8600000143051148,0.8299999833106995,0.7699999809265137,0.699999988079071],11,10));
motor_data.eff_torque_break = [1.3,2.7,5.4,7.9,10.4,12.5,14.4,16.0,17.4,18.5,19.6];
motor_data.eff_rpm_break = [500,1000,2000,3000,4000,6000,10000,12000,15000,19000];
motor_data.totpow = times(motor_data.eff,(motor_data.eff_torque_break'*motor_data.eff_rpm_break));
%motor_data.inertia = 2.74;         % [kg/cm^2]
motor_data.inertia = 0.000274;      % [kg/m^2]
motor_data.mass = 3.55;              % [kg]

%inline steer tables
% SteeringWhAngle = [-105 -99.75 -94.5 -89.25 -84 -78.75 -73.5 -68.25 -63 -57.75 -52.5 -47.25 -42 -36.75 -31.5 -26.25 -21.0 -15.75 -10.5 -5.25 0.0 5.25 10.50 15.75 21.00 26.25 31.50 36.75 42.00 47.25 52.50 57.75  63.00 68.25 73.50 78.75 84.00 89.25 94.50 99.75 105.00];

%2017:
% SteeringAngleFL = [-24.129 -22.950 -21.772 -20.593 -19.413 -18.232 -17.050 -15.865 -14.677 -13.485 -12.290 -11.091 -9.886 -8.676 -7.460 -6.237 -5.007 -3.769 -2.522 -1.266 0.000 1.277 2.566 3.868 5.183 6.513 7.859 9.221 10.602 12.003 13.425 14.869 16.339 17.835 19.361 20.918 22.511 24.141 25.814 27.534 29.305];
% SteeringAngleFR = [-29.3050 -27.5340 -25.8140 -24.1410 -22.5110 -20.9180 -19.3610 -17.8350 -16.3390 -14.8690 -13.4250 -12.0030 -10.6020 -9.2210 -7.8590 -6.5130 -5.1830 -3.8680 -2.5660 -1.2770 0.000 1.266 2.522 3.769 5.007 6.237 7.460 8.676 9.886 11.091 12.290 13.485 14.677 15.865 17.050 18.232 19.413 20.593 21.772 22.950 24.129];

%2018:
SteeringAngleFL = [-23.80 -22.65 -21.50 -20.34 -19.18 -18.03 -16.86 -15.70 -14.53 -13.35 -12.18 -10.99 -9.80 -8.61 -7.40 -6.19 -4.97 -3.75 -2.51 -1.26 0 1.27 2.56 3.85 5.17 6.50 7.84 9.21 10.59 12.00 13.42 14.88 16.36 17.87 19.41 20.99 22.61 24.26 25.97 27.73 29.54];
SteeringAngleFR = [-29.54 -27.73 -25.97 -24.26 -22.61 -20.99 -19.41 -17.87 -16.36 -14.88 -13.42 -12.00 -10.59 -9.21 -7.84 -6.50 -5.17 -3.85 -2.56 -1.27 0 1.26 2.51 3.75 4.97 6.19 7.40 8.61 9.80 10.99 12.18 13.35 14.53 15.70 16.86 18.03 19.18 20.34 21.50 22.65 23.80];



display('loading m-file complete : TV_2017_parameters.m ')

