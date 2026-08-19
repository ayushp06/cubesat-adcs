function params = modeManagerParams()
% MODEMANAGERPARAMS Deterministic ADCS transition thresholds.
    params.detumbleEntryRate=deg2rad(5);
    params.detumbleExitRate=deg2rad(0.5);
    params.desaturationEntryFraction=0.85;
    params.desaturationExitFraction=0.50;
end
