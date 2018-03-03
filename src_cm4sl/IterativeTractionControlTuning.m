% Iterative Tuning Of Traction Control

duration = (500:100:1500);
gain     = (0.5:0.05:1);
iteration = 1;
dataStorage = zeros(3 , (length(duration)*length(gain)));


for i = 1:(length(gain))
    for j = 1:(length(duration))
        disp(sprintf('starting simulation %f' , iteration));
        %load('Buses_2017.mat');
        tractionControl = struct;
        %-------------------------------------------------------------------------------------------
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

        tractionControl.startValueSlip = single(gain(i)); %scales the torque from 1 to startValueSlip at slip detection.
        tractionControl.startValueLock = single(1); %scales the torque from 1 to startValueLock at lock detection.
        tractionControl.riseTime  = single(duration(j)); %iterations from 0 to 1;

        %sensitivity parameters for the slip detection
        tractionControl.slip_rate_pos_lim = single(0.25);     %limit for slip, that corresponds to wheelslip. Can be moved up and down to make traction control more agressive.
        tractionControl.slip_rate_neg_lim = single(-0.30);    % limit of negative slip that corresponds to wheels "locking", can be moved up and down to make traction control more/less agressive.
        tractionControl.omega_der_slip_lim = single(2.5 *(9.81/0.225));  %Upper limit for plausible wheel angular acceleration, without wheelspin.
        tractionControl.omega_der_lock_lim = single(-4.5*(9.81/0.225));  %lower limit for plausible wheel angular acceleration, without "wheellock".
        
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
        %----------------------------------------------------------------
        sim('TV_2017.slx');
        pause(5);
        load('E:\CM_Projects\R18_TV\src_cm4sl\Mat_files\VD_Vhcl&Driver.mat');
        time = Vhcl.TriggerPointTime.Time(end);
        dataStorage(: , iteration) = [time  gain(i)  duration(j)]'
        iteration = iteration +1;
    end   
end