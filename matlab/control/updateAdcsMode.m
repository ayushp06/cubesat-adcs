function state = updateAdcsMode(state,status,params)
% UPDATEADCSMODE Priority-ordered ADCS mode transitions with hysteresis.
    if status.fault
        state.mode="fault"; return;
    end
    if strcmp(state.mode,"fault")
        if status.faultReset, state.mode="initialization"; end
        return;
    end
    if strcmp(state.mode,"initialization")
        if ~status.initializationComplete, return; end
        if status.bodyRate>params.detumbleEntryRate, state.mode="detumble"; else, state.mode="safe"; end
        return;
    end
    if strcmp(state.mode,"detumble")
        if status.bodyRate<=params.detumbleExitRate, state.mode="safe"; end
        return;
    end
    if status.safeRequested || ~status.estimatorValid
        state.mode="safe"; return;
    end
    if strcmp(state.mode,"desaturation")
        if status.wheelSpeedFraction<=params.desaturationExitFraction
            state.mode=state.previousOperationalMode;
        end
        return;
    end
    if status.wheelSpeedFraction>=params.desaturationEntryFraction
        state.previousOperationalMode=state.mode;
        state.mode="desaturation"; return;
    end
    if status.slewRequested
        state.mode="slew"; return;
    end
    if strcmp(state.mode,"slew") && ~status.slewComplete, return; end
    if status.nominalRequested, state.mode="nominal"; else, state.mode="safe"; end
end
