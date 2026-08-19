function state = initializeAdcsMode()
% INITIALIZEADCSMODE Initial deterministic flight-mode state.
    state.mode="initialization";
    state.previousOperationalMode="safe";
end
