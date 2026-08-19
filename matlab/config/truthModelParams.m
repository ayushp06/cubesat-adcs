function model = truthModelParams()
% TRUTHMODELPARAMS Switches for independently validated truth-model effects.
    model.includeJ2 = true;
    model.includeDrag = true;
    model.includeSolarPressure = true;
    model.includeGravityGradient = true;
    model.includeAerodynamicTorque = true;
    model.includeSolarPressureTorque = true;
    model.includeMagneticTorque = true;
end
