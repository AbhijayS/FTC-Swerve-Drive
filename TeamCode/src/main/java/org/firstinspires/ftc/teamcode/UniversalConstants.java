package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

public class UniversalConstants {

    public static final String vuforiaLicenceKey = "AfWYo6H/////AAABmQhOGpga9kzugmfBYrjJDQs2b+vIsT5Pjplu+VgGrxevhveFPbYqHKZeHrJKY1bhrSNzeogphVwdC2ndsu9sn9WZ/yOrOhQ9s8egFXfDzkPN6uhzqVo5A3qGAPK8CPqAcdraaJxMKINGPo+aBqbzELtMl+kX1BkUfbapRDYIBRS+oySt71kDzykpd0Iy7NgJGU5xfsEm8fJm9xWVTFV7IsChCAKeo7bmBUtcpIVR6smjqRT9noJcmWv2aP9J1C2CcW766ptLy0lKIl08M+S2uY8XYHjK2kBpOnHvkSrzsnrROgE/g7V4xDiFCEFBjk1w4qAcO5veTVzbhqxxSmWMQkb7JiORCUCZ+Dz5F8bYW5zY";

    public enum Status {
        DEBUGGING, TESTING, RELEASE
    }

    public enum SwerveState {
        PATH_FOLLOWING,
        PATH_FOLLOWING_COMPLETE,
        HUMAN_INPUT
    }

    public static double roundTo2DecimalPlaces(double a) {
        return Math.round(a * 100.0) / 100.0;
    }

    public enum MotionState {
        STOPPED,
        IN_MOTION,
        ACCELERATING,
        DECELERATING;
    }

    public enum Debugging {
        RX("Robot X"),
        RY("Robot Y"),
        WA("Module0"),
        HEADING("Heading"),
        DISTANCE("Distance Delta"),
        VELOCITY("Velocity"),
        TRACE("Old TP X"),
        SEG("Segment"),
        PX("Tracking Point X"),
        PY("Tracking Point Y"),
        ERROR("Error"),
        CORRECTION("Correction"),
        PATH("Path");

        private final String name;

        Debugging(String name) {
            this.name = name;
        }

        @Override
        public String toString() {
            return name;
        }

        public static List<String> getDebuggingMarkers() {
            ArrayList<String> result = new ArrayList<>();
            for (Debugging d : Debugging.values()) {
                result.add(d.toString());
            }
            return result;
        }

    }

    public static final DecimalFormat decimalFormat = new DecimalFormat("#.##");


    public enum MotorDirection {
        FORWARD(1.0f, "Forward"),
        REVERSE(-1.0f, "Reverse");

        private final float sign;
        private final String name;
        private final String abbreviation;

        MotorDirection(float sign, String name) {
            this.sign = sign;
            this.name = name;
            if (name.length() > 0)
                abbreviation = name.substring(0, 1);
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

        public MotorDirection getOpposite() {
            if (this == MotorDirection.FORWARD)
                return REVERSE;
            return FORWARD;
        }

        public double assignDirection(double value) {
            return Math.copySign(value, sign);
        }
    }

    static final int CYCLE_MS = 50;
    public static final Status ROBOT_STATUS = Status.TESTING;
    public static final double ROBOT_WIDTH = 14.5; // INCHES
    public static final double ROBOT_LENGTH = 15; // INCHES
    public static final double ROBOT_MAX_SPEED = 0.05;
    static final double ROBOT_COM_X = 0; // offset from (0,0)
    static final double ROBOT_COM_Y = 0; // offset from (0,0)
    static final double PI = Math.PI;
    public static final double HALF_PI = PI / 2;

    static final double TURN_FACTOR = 2; // Tune the robot's speed at turns (1 - 5)
    public static final int servoRange = 270;      // 225° to -45° on the Unit Circle
    public static final double servoSpeed = 0.16 / 60;  // sec/1° @ 5V TODO: Update speed for 6V input from the servo power module
    public static final int servoDefaultAngle = 90;
    public static final int ticksPerRevolution = 560;
    public static final int wheelDiameter = 3; //inches
    public static final double wheelCircumference = wheelDiameter * Math.PI; //inches

    public static final double driveGearRatio = 24.0 / 18.0; // Output rate : Input rate
    public static final double kS =   5; // Steering gain (path following only)
    public static final double kP =   0; // Proportional gain
    public static final double kI =   0; // Integral gain
    public static final double kD =   0; // Derivative gain


    public static final PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(500, 2500);



    public enum ModuleConfig {
        MODULE_ZERO     ("Module ZERO",     (-ROBOT_WIDTH/2)-ROBOT_COM_X,(ROBOT_LENGTH/2)-ROBOT_COM_Y,     false,  "FLS",    0        ,  Servo.Direction.REVERSE,    "FLM",  MotorDirection.REVERSE)  ,
        MODULE_ONE      ("Module ONE",      (ROBOT_WIDTH/2)-ROBOT_COM_X,(ROBOT_LENGTH/2)-ROBOT_COM_Y,      false,  "FRS",   -3.07    ,  Servo.Direction.FORWARD,    "FRM", MotorDirection.FORWARD)  ,
        MODULE_TWO      ("Module TWO",      (ROBOT_WIDTH/2)-ROBOT_COM_X,(-ROBOT_LENGTH/2)-ROBOT_COM_Y,     false,  "RRS",    2.92     ,  Servo.Direction.FORWARD,    "RRM",  MotorDirection.FORWARD)  ,
        MODULE_THREE    ("Module THREE",    (-ROBOT_WIDTH/2)-ROBOT_COM_X,(-ROBOT_LENGTH/2)-ROBOT_COM_Y,    false,  "RLS",     -4.43    ,  Servo.Direction.FORWARD,    "RLM",   MotorDirection.REVERSE)  ;

        public final String                     moduleName;
        public final String                     servoID;
        public final double                     servoDelta;
        public final Servo.Direction            servoDirection;
        public final boolean                    isModuleDisabled;
        public final String                     motorID;
        public final double                     x, y;

        public final MotorDirection    motorDirection;
        ModuleConfig(String module_name, double x, double y, boolean is_module_disabled, String servo_id, double servo_delta, Servo.Direction servo_direction, String motor_id, MotorDirection motor_direction) {
            this.moduleName         = module_name;
            this.isModuleDisabled   = is_module_disabled;
            this.servoID            = servo_id;
            this.servoDelta         = servo_delta;
            this.servoDirection     = servo_direction;
            this.motorID            = motor_id;
            this.motorDirection     = motor_direction;
            this.x = x;
            this.y = y;
        }

    }

    /**
     * @param a - any angle in degrees in the range (-infinity to infinity)
     * @return Normalizes the angle in the range (-180 to 180)
     */
    public static double clipAngle(double a) {
        return Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(a)), Math.cos(Math.toRadians(a))));
    }

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
}
