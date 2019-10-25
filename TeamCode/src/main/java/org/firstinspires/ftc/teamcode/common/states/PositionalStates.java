package org.firstinspires.ftc.teamcode.common.states;

public enum PositionalStates {

    FULL(42),
    THREE_QUARTERS(0.75 * 42),
    HALF(0.5 * 42),
    QUARTER(.25);

    public final double height;
    public final double runPower;


    PositionalStates(double height) {
        this.height = height;
        runPower = .5;
    }
}
