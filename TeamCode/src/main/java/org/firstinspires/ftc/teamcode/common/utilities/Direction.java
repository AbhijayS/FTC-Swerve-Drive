package org.firstinspires.ftc.teamcode.common.utilities;

public enum Direction {
    FORWARD(1.0f,"Forward"),
    REVERSE(-1.0f,"Reverse"),
    RIGHT(1.0f, "Right"),
    LEFT(-1.0f, "Left"),
    UNKOWN(0.0f, "Unknown");

    private final float sign;
    private final String name;
    private final String abbreviation;

    Direction(float sign, String name) {
        this.sign = sign;
        this.name = name;
        if (name.length() > 0)
            abbreviation = name.substring(0,1);
        else
            abbreviation = "";
    }

    @Override
    public String toString() {
        return name;
    }

    public String getAbbreviation() {
        return abbreviation;
    }

    public float getSign() {
        return sign;
    }

    public Direction getOpposite() {
        switch (this) {
            case LEFT:
                return RIGHT;
            case RIGHT:
                return LEFT;
            case FORWARD:
                return REVERSE;
            default:
                return FORWARD;
        }
    }

    public double assignDirection(double value) {
        return Math.copySign(value, sign);
    }
}