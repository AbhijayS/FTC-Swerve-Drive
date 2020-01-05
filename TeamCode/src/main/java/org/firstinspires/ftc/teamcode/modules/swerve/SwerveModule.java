package org.firstinspires.ftc.teamcode.modules.swerve;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Stopwatch;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.AUTO_MAX_SPEED;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.ROBOT_STATUS;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.driveGearRatio;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.pwmRange;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.ticksPerRevolution;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.wheelCircumference;

public class SwerveModule {
    private final int MAX = 315;
    private final int MIN = 225;

    private UniversalConstants.ModuleConfig config;
    private SwerveDrive swerveDrive;
    public Servo turnServo;
    public DcMotor driveMotor;
    private Debugger debugger;

    private UniversalConstants.MotorDirection motorDirection;
    private double servoPosition;
    private double internalServoPosition;
    private double delta;
    private double normal, opposite;
    private STYLE style;
    public enum STYLE {
        NORMAL, OPPOSITE
    }
    public boolean ONE_EIGHTY = false;
    private Stopwatch timer = new Stopwatch();

    public SwerveModule(HardwareMap hardwareMap, UniversalConstants.ModuleConfig config, SwerveDrive swerveDrive) {
        this.config = config;
        this.swerveDrive = swerveDrive;

        // servo settings
        this.turnServo = hardwareMap.servo.get(config.servoID);
        this.delta = config.servoDelta;
        this.normal = normalize(90 + delta);
        this.opposite = normalize(-90 + delta);
        this.style = config.style;
        turnServo.setDirection(config.servoDirection);
        if (turnServo.getController() instanceof ServoControllerEx) {
            // Confirm its an extended range servo controller before we try to set to avoid crash
            ServoControllerEx theControl = (ServoControllerEx) turnServo.getController();
            theControl.setServoPwmRange(turnServo.getPortNumber(), pwmRange);
        }

        // motor settings
        this.driveMotor = hardwareMap.dcMotor.get(config.motorID);
        motorDirection = config.motorDirection;
        driveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        swivel(90);
        setPower(0);
    }

    ;

    public static int convertInchesToTicks(double inches) {
        return (int) Math.round(inches * ticksPerRevolution / wheelCircumference / driveGearRatio);
    }

    public double swivel(double targetAngle) {
        targetAngle = normalize(targetAngle);
        resetMotorDirection();

        // NORMAL
        if (style == STYLE.NORMAL) {
            // opposite angle is closer to the driver input
            if (Math.abs(angleDelta(targetAngle, opposite)) < 90) {
                opposite = targetAngle;
                normal = normalize(targetAngle + 180);
                // normal motors flip direction
                reverseMotorDirection();
            }
            // normal angle is closer to the driver input
            else {
                normal = targetAngle;
                opposite = normalize(normal + 180);
                // opposite motors flip direction
            }

            normal = normalize(normal + delta);
            opposite = normalize(opposite + delta);

            // normal angle is in the dead-zone
            if (normal > 225 && normal < 315) {
                // swivel all to opposite angle
                turnServo.setPosition(scaleAngle(opposite));
                // normal motors flip direction
                reverseMotorDirection();
            }
            // none are in the dead-zone
            else {
                // swivel to the respective angles
                turnServo.setPosition(scaleAngle(normal));
            }
        }

        // OPPOSITE
        else {
            // opposite angle is closer to the driver input
            if (Math.abs(angleDelta(targetAngle, opposite)) < 90) {
                opposite = targetAngle;
                normal = normalize(targetAngle + 180);
                // normal motors flip direction
            }
            // normal angle is closer to the driver input
            else {
                normal = targetAngle;
                opposite = normalize(normal + 180);
                // opposite motors flip direction
                reverseMotorDirection();
            }

            normal = normalize(normal + delta);
            opposite = normalize(opposite + delta);

            // opposite angle is in the dead-zone
            if (opposite > 225 && opposite < 315) {
                // swivel all to normal angle
                turnServo.setPosition(scaleAngle(normal));
                // opposite motors flip direction
                reverseMotorDirection();
            }
            // none are in the dead-zone
            else {
                // swivel to the respective angles
                turnServo.setPosition(scaleAngle(opposite));
            }
        }

//        if (angleDelta(targetAngle,servoPosition) >= 180) {
//            ONE_EIGHTY = true;
//            timer.reset();
//            timer.start();
//        }
//
//        // timer control
//        if (ONE_EIGHTY) {
//            if (timer.millis() >= 800) {
//                ONE_EIGHTY = false;
//                timer.reset();
//            }
//        }

        servoPosition = targetAngle;
        return 0;
    }


    // turn methods
    private double normalize(double angle) {
        // 0 - 360
        return ((angle % 360) + 360) % 360;
    }

    private double scaleAngle(double angle) {
        // 225 ==> 0
        // 315 ==> 1
        if (angle >= 315)
            angle -= 360;
        return (225 - angle) / 270;
    }

    private double angleDelta(double target, double source) {
        double diff = target - source;
        diff += (diff > 180) ? -360 : (diff < -180) ? 360 : 0;
        return diff;
    }

    public double getServoPosition() {
        return servoPosition;
    }

    public void setPosition(double pos) {
        turnServo.setPosition(pos);
    }

    // drive methods
    public void setPower(double power) {
        if (ONE_EIGHTY) {
            power = 0;
        } else {
            power = Range.clip(power, 0, AUTO_MAX_SPEED);
            power = motorDirection.assignDirection(power);
        }

        if (ROBOT_STATUS != UniversalConstants.Status.DEBUGGING)
            driveMotor.setPower(power);
    }

    public void reverseMotorDirection() {
        motorDirection = motorDirection.getOpposite();
    }

    public void resetMotorDirection() {
        motorDirection = config.motorDirection;
    }

    public UniversalConstants.MotorDirection getMotorDirection() {
        return this.motorDirection;
    }

    public void enablePID() {
        driveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void disablePID() {
        driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void movePID(int position, double power) {
        double wheelStamp = swerveDrive.swerveKinematics.wheelStamps[config.ordinal()];
        int ticks = convertInchesToTicks(wheelStamp);
        int target = ticks + (int) motorDirection.assignDirection(position);
        driveMotor.setTargetPosition(target);
        driveMotor.setPower(power);
    }

    // general methods
    public String toString() {
        return config.moduleName;
    }
}
