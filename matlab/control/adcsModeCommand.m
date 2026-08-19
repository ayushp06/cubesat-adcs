function command = adcsModeCommand(mode)
% ADCSMODECOMMAND Map flight mode to enabled actuation and guidance.
    command.reactionWheels=false; command.magnetorquers=false; command.guidance="none";
    if strcmp(mode,"detumble")
        command.magnetorquers=true; command.guidance="rate";
    elseif strcmp(mode,"safe")
        command.reactionWheels=true; command.guidance="safe";
    elseif strcmp(mode,"nominal")
        command.reactionWheels=true; command.guidance="nominal";
    elseif strcmp(mode,"slew")
        command.reactionWheels=true; command.guidance="slew";
    elseif strcmp(mode,"desaturation")
        command.reactionWheels=true; command.magnetorquers=true; command.guidance="hold";
    end
end
