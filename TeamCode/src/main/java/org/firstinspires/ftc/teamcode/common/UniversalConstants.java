package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

public class UniversalConstants {



    /*
     * Work Flow:
     * TESTING => DEBUGGING => TESTING => DEBUGGING => ...
     * TESTING => RELEASE
     */

    public static final String vuforiaLicenceKey = "AfWYo6H/////AAABmQhOGpga9kzugmfBYrjJDQs2b+vIsT5Pjplu+VgGrxevhveFPbYqHKZeHrJKY1bhrSNzeogphVwdC2ndsu9sn9WZ/yOrOhQ9s8egFXfDzkPN6uhzqVo5A3qGAPK8CPqAcdraaJxMKINGPo+aBqbzELtMl+kX1BkUfbapRDYIBRS+oySt71kDzykpd0Iy7NgJGU5xfsEm8fJm9xWVTFV7IsChCAKeo7bmBUtcpIVR6smjqRT9noJcmWv2aP9J1C2CcW766ptLy0lKIl08M+S2uY8XYHjK2kBpOnHvkSrzsnrROgE/g7V4xDiFCEFBjk1w4qAcO5veTVzbhqxxSmWMQkb7JiORCUCZ+Dz5F8bYW5zY";
    public enum Status {
        DEBUGGING, TESTING, RELEASE
    }

    public static final DecimalFormat decimalFormat = new DecimalFormat("#.##");

    public enum Debugging {
        RX("Robot X"),
        RY("Robot Y"),
        PX("Tracking Pose X"),
        PY("Tracking Pose Y"),
        HEADING("Heading"),
        VELOCITY("Velocity"),
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

    public enum MotorDirection {
        FORWARD(1.0f,"Forward"),
        REVERSE(-1.0f,"Reverse");

        private final float sign;
        private final String name;
        private final String abbreviation;

        MotorDirection(float sign, String name) {
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

        public MotorDirection getOpposite() {
            if (this == MotorDirection.FORWARD)
                return REVERSE;
            return FORWARD;
        }

        public double assignDirection(double value) {
            return Math.copySign(value, sign);
        }
    }

    public enum ModuleConfig {
        MODULE_ZERO     ("Module ZERO",     (-ROBOT_WIDTH/2)-ROBOT_COM_X,(ROBOT_LENGTH/2)-ROBOT_COM_Y,     false,  "fls",    -14.5        ,  Servo.Direction.FORWARD,    "flm",  MotorDirection.REVERSE)  ,
        MODULE_ONE      ("Module ONE",      (ROBOT_WIDTH/2)-ROBOT_COM_X,(ROBOT_LENGTH/2)-ROBOT_COM_Y,      false,  "frs",   0.8    ,  Servo.Direction.FORWARD,    "frm", MotorDirection.FORWARD)  ,
        MODULE_TWO      ("Module TWO",      (ROBOT_WIDTH/2)-ROBOT_COM_X,(-ROBOT_LENGTH/2)-ROBOT_COM_Y,     false,  "rrs",    -10.23     ,  Servo.Direction.FORWARD,    "rrm",  MotorDirection.FORWARD)  ,
        MODULE_THREE    ("Module THREE",    (-ROBOT_WIDTH/2)-ROBOT_COM_X,(-ROBOT_LENGTH/2)-ROBOT_COM_Y,    false,  "rls",     -4.27    ,  Servo.Direction.FORWARD,    "rlm",   MotorDirection.REVERSE)  ;

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

        public double rawX() {
            return x+ROBOT_COM_X;
        }

        public double rawY() {
            return y+ROBOT_COM_Y;
        }
    }

    public enum JewelSwatterConfig {
        LEFT("JSLA1", "JSLA2", Servo.Direction.FORWARD, Servo.Direction.REVERSE, 0),
        RIGHT("JSRA1", "JSRA2", Servo.Direction.FORWARD, Servo.Direction.FORWARD, 0.1);

        public String nameAxis1, nameAxis2;
        public double delta;
        public Servo.Direction directionAxis1, directionAxis2;

        JewelSwatterConfig(String axis1, String axis2, Servo.Direction directionAxis1, Servo.Direction directionAxis2, double delta) {
            this.nameAxis1 = axis1;
            this.nameAxis2 = axis2;
            this.directionAxis1 = directionAxis1;
            this.directionAxis2 = directionAxis2;
            this.delta = delta;
        }
    }

    public static final int     CYCLE_MS            =   50;
    public static final Status  ROBOT_STATUS        =   Status.TESTING;
    public static final double  ROBOT_WIDTH         =   14.75; // INCHES
    public static final double  ROBOT_LENGTH        =   11.8; // INCHES
    public static final double  AUTO_MAX_SPEED =   1;
    public static final double  TELEOP_MAX_SPEED =   0.5;
    public static final double  ROBOT_COM_X         =   0; // offset from (0,0)
    public static final double  ROBOT_COM_Y         =   1; // offset from (0,0)
    public static final double  PI                  =   Math.PI;
    public static final double  HALF_PI             =   PI/2;

    public static final double  TURN_FACTOR         =   2; // Tune the robot's speed at turns (1 - 5)
    public static final int     servoRange          = 270;      // 225° to -45° on the Unit Circle
    public static final double  servoSpeed          = 0.2/60;  // sec/1° @ 5V TODO: Update speed for 6V input from the servo power module
    public static final int     servoDefaultAngle   = 90;
    public static final int     ticksPerRevolution  = 560;
    public static final int     wheelDiameter       = 3; //inches
    public static final double  wheelCircumference  = wheelDiameter*Math.PI; //inches
    public static final double  driveGearRatio      = 24.0/18.0; // Output rate : Input rate

    // various hardware names
    public static final String expansionHubBottom = "Expansion Hub 1";
    public static final String expansionHubTop = "Expansion Hub 2";
    public static final String clampServoTop = "CST";
    public static final String clampServoBottom = "CSB";
    public static final String clampServo = "CS";
    public static final String leftStop = "leftStop";
    public static final String rightStop = "rightStop";
    public static final String liftMotorA = "liftA";
    public static final String liftMotorB = "liftB";
    public static final String webcamName = "Webcam 1";


    public static double kS =   6; // Steering gain (path following only)
    public static final double kP =   0.5/90; // Proportional gain
    public static final double kI =   0.00001; // Integral gain
    public static final double kD =   0; // Derivative gain
    public static final double tolerance = 1; // degrees or inches

    public static final PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(500, 2500);



    /*
    Other Helpful mathematical functions
     */

    /**
     * @param a - any angle in degrees in the range (-infinity to infinity)
     * @return Normalizes the angle in the range (-180 to 180)
     */
    public static double clipAngle(double a) {
        return Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(a)), Math.cos(Math.toRadians(a))));
    }
    public static boolean oppositeSigns(double a, double b) {
        return a*b < 0;
    }
    public static double roundTo2DecimalPlaces(double a) {
        return Math.round(a * 100.0) / 100.0;
    }
}
