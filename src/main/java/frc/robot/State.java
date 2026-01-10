package main.java.frc.robot;

import com.google.common.collect.ImmutableMap;
import frc.spectrumLib.Telemetry;
import java.util.Map;
import java.util.function.BooleanSupplier;
import lombok.Setter;

public enum State {
    REHOME,

    PRE_AMP,
    AMP,

    PRE_SHOT,
    SHOT,

    INTAKE,
    EJECT;

    private State() {}

    private static final ImmutableMap<State, State> scoreSequence = 
        ImmutableMap.ofEntries(
            Map.entry(PRE_SHOT, SHOT),
            Map.entry(PRE_AMP, AMP)
        );
}