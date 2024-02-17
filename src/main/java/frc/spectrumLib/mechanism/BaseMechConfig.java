package frc.spectrumLib.mechanism;

public class BaseMechConfig {

    /* Which canbus the mechanism is on */
    public String CANBUS = "rio";

    /* CAN ID of the mechanism */
    public int ID = 0;

    /* Gear ratio of mechanism motor */
    public double GEAR_RATIO = 1;

    /** Motor invert: False is CounterClockwise_Positive, True is Clockwise_Positive */
    public boolean INVERTED = false;

    /** Starting position in rotations */
    public double STARTING_POSITION = 0;

    public BaseMechConfig withCANbusName(String CANbusName) {
        this.CANBUS = CANbusName;
        return this;
    }

    public BaseMechConfig withId(int id) {
        this.ID = id;
        return this;
    }

    public BaseMechConfig withGearRatio(double gearRatio) {
        this.GEAR_RATIO = gearRatio;
        return this;
    }

    public BaseMechConfig withInverted(boolean inverted) {
        this.INVERTED = inverted;
        return this;
    }

    public BaseMechConfig withStartingPos(double rotations) {
        this.STARTING_POSITION = rotations;
        return this;
    }
}
