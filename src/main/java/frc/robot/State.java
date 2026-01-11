package frc.robot;

import com.google.common.collect.ImmutableMap;
import java.util.Map;

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