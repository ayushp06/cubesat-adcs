function results=runFaultScenario()
% RUNFAULTSCENARIO Exercise estimator, wheel, and latched-fault responses.
    params=modeManagerParams(); state=initializeAdcsMode(); status=struct( ...
        "fault",false,"faultReset",false,"initializationComplete",true, ...
        "bodyRate",0,"safeRequested",false,"estimatorValid",true, ...
        "wheelSpeedFraction",0,"slewRequested",false,"slewComplete",true,"nominalRequested",true);
    modes=cell(1,8);
    state=updateAdcsMode(state,status,params); modes{1}=state.mode;
    state=updateAdcsMode(state,status,params); modes{2}=state.mode;
    status.estimatorValid=false; state=updateAdcsMode(state,status,params); modes{3}=state.mode;
    status.estimatorValid=true; status.wheelSpeedFraction=.9; state=updateAdcsMode(state,status,params); modes{4}=state.mode;
    status.fault=true; state=updateAdcsMode(state,status,params); modes{5}=state.mode;
    status.fault=false; state=updateAdcsMode(state,status,params); modes{6}=state.mode;
    status.faultReset=true; state=updateAdcsMode(state,status,params); modes{7}=state.mode;
    status.faultReset=false; status.bodyRate=deg2rad(8); state=updateAdcsMode(state,status,params); modes{8}=state.mode;
    expected={"safe","nominal","safe","desaturation","fault","fault","initialization","detumble"};
    assert(isequal(modes,expected)); results.modes=modes; results.passed=true;
end
