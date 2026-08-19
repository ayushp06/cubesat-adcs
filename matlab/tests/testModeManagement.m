clear; clc;
testDir=fileparts(mfilename("fullpath"));
addpath(fullfile(testDir,"..","config"),fullfile(testDir,"..","control"));
p=modeManagerParams(); state=initializeAdcsMode();
s=struct("fault",false,"faultReset",false,"initializationComplete",false, ...
    "bodyRate",0,"safeRequested",false,"estimatorValid",true, ...
    "wheelSpeedFraction",0,"slewRequested",false,"slewComplete",false,"nominalRequested",true);
state=updateAdcsMode(state,s,p); assert(strcmp(state.mode,"initialization"));
s.initializationComplete=true; s.bodyRate=deg2rad(8); state=updateAdcsMode(state,s,p); assert(strcmp(state.mode,"detumble"));
s.bodyRate=deg2rad(1); state=updateAdcsMode(state,s,p); assert(strcmp(state.mode,"detumble"));
s.bodyRate=deg2rad(0.4); state=updateAdcsMode(state,s,p); assert(strcmp(state.mode,"safe"));
state=updateAdcsMode(state,s,p); assert(strcmp(state.mode,"nominal"));
s.slewRequested=true; state=updateAdcsMode(state,s,p); assert(strcmp(state.mode,"slew"));
s.slewRequested=false; state=updateAdcsMode(state,s,p); assert(strcmp(state.mode,"slew"));
s.slewComplete=true; state=updateAdcsMode(state,s,p); assert(strcmp(state.mode,"nominal"));
s.wheelSpeedFraction=.9; state=updateAdcsMode(state,s,p); assert(strcmp(state.mode,"desaturation"));
s.wheelSpeedFraction=.7; state=updateAdcsMode(state,s,p); assert(strcmp(state.mode,"desaturation"));
s.wheelSpeedFraction=.4; state=updateAdcsMode(state,s,p); assert(strcmp(state.mode,"nominal"));
s.fault=true; state=updateAdcsMode(state,s,p); assert(strcmp(state.mode,"fault"));
s.fault=false; state=updateAdcsMode(state,s,p); assert(strcmp(state.mode,"fault"));
s.faultReset=true; state=updateAdcsMode(state,s,p); assert(strcmp(state.mode,"initialization"));

rw.A=[1,0,0,1/sqrt(3);0,1,0,1/sqrt(3);0,0,1,1/sqrt(3)];
torque=[1;-2;3]*1e-4; motors=allocateReactionWheelTorque(torque,rw);
assert(norm(-rw.A*motors-torque)<1e-15 && numel(motors)==4);
rw.A=rw.A(:,2:4); motors=allocateReactionWheelTorque(torque,rw);
assert(norm(-rw.A*motors-torque)<1e-15);
allModes={"initialization","detumble","safe","nominal","slew","desaturation","fault"};
for index=1:numel(allModes)
    command=adcsModeCommand(allModes{index}); assert(islogical(command.reactionWheels) && islogical(command.magnetorquers));
end
faultCommand=adcsModeCommand("fault"); detumbleCommand=adcsModeCommand("detumble");
assert(~faultCommand.reactionWheels && detumbleCommand.magnetorquers);
disp("ALL MODE MANAGEMENT TESTS PASSED");
