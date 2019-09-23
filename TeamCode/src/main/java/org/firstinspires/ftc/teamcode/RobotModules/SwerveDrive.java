package org.firstinspires.ftc.teamcode.RobotModules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants.SwerveState;

import static org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants.Debugging.CORRECTION;
import static org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants.Debugging.ERROR;
import static org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants.Debugging.PATH;
import static org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants.Debugging.PX;
import static org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants.Debugging.PY;
import static org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants.Debugging.VELOCITY;
import static org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants.Debugging.WA;
import static org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants.HALF_PI;
import static org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants.ROBOT_LENGTH;
import static org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants.ROBOT_MAX_SPEED;
import static org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants.ROBOT_STATUS;
import static org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants.ROBOT_WIDTH;
import static org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants.Status.RELEASE;
import static org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants.SwerveState.PATH_FOLLOWING;
import static org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants.SwerveState.PATH_FOLLOWING_COMPLETE;
import static org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants.kS;
import static org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants.roundTo2DecimalPlaces;

public class SwerveDrive {

    /* Private OpMode members. */
    SwerveModule module0;
    SwerveModule module1;
    SwerveModule module2;
    SwerveModule module3;
    private LinearOpMode linearOpMode;
    private Path path = null;
    private double IMU_ZERO;
    private Point CoM;
    private Point trackingPoint; // Point on the path to be tracked
    private int splineSegment;
    private Debugger debugger;
    private UniversalConstants.SwerveState swerveState;
    private UniversalConstants.MotionState motionState;
    public SwerveKinematics swerveKinematics;

    public SwerveDrive(LinearOpMode l, Debugger debugger) {
        linearOpMode = l;
        this.debugger = debugger;
        this.swerveKinematics = new SwerveKinematics(l, debugger, this);
        this.swerveState = UniversalConstants.SwerveState.HUMAN_INPUT;
        this.motionState = UniversalConstants.MotionState.STOPPED;

        // Define and Initialize Swerve Modules
        module0 = new SwerveModule(l, UniversalConstants.ModuleConfig.MODULE_ZERO.MODULE_ZERO);
        module1 = new SwerveModule(l, UniversalConstants.ModuleConfig.MODULE_ONE);
        module2 = new SwerveModule(l, UniversalConstants.ModuleConfig.MODULE_TWO);
        module3 = new SwerveModule(l, UniversalConstants.ModuleConfig.MODULE_THREE);

        CoM = new Point();

        this.IMU_ZERO = swerveKinematics.getIMU_ZERO();
        linearOpMode.telemetry.addLine(String.format("Swerve Drive Calibrated in %s Status", ROBOT_STATUS));
    }

    public void requestState(UniversalConstants.SwerveState swerveState) {
        this.swerveState = swerveState;
    }

    public void turnDrive(Gamepad g) {

        // TODO: Simplify code

        if (swerveState == PATH_FOLLOWING || swerveState == PATH_FOLLOWING_COMPLETE) {
            return;
        }

        double x_left = g.left_stick_x;
        double y_left = -g.left_stick_y;

        double OMEGA = Range.scale(g.right_stick_x, 0, 1, 0, ROBOT_MAX_SPEED); // Rotational speed: Clockwise is positive and Anti-Clockwise is negative
        double STR = Range.scale(Math.abs(Math.hypot(x_left, y_left)), 0, 1, 0, ROBOT_MAX_SPEED); // Strafing speed
        double STR_ANGLE = Math.toDegrees(Math.atan2(y_left, x_left)); // Strafing angle
        double corner0 = Math.atan2(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2);
        double corner1 = corner0 - (HALF_PI);
        double corner2 = corner1 - (HALF_PI);
        double corner3 = corner2 - (HALF_PI);

        /* Make angles field-oriented */
        double delta = swerveKinematics.getYaw();
        x_left = STR * Math.cos(Math.toRadians(STR_ANGLE - delta)); // TODO: generalize gyro angles so they don't depend on the hub's orientation
        y_left = STR * Math.sin(Math.toRadians(STR_ANGLE - delta));
        STR_ANGLE = Math.toDegrees(Math.atan2(y_left, x_left));

        double swivel0 = Math.toDegrees(Math.atan2((y_left + Math.sin(corner0) * OMEGA), (x_left + Math.cos(corner0) * OMEGA)));
        double swivel1 = Math.toDegrees(Math.atan2((y_left + Math.sin(corner1) * OMEGA), (x_left + Math.cos(corner1) * OMEGA)));
        double swivel2 = Math.toDegrees(Math.atan2((y_left + Math.sin(corner2) * OMEGA), (x_left + Math.cos(corner2) * OMEGA)));
        double swivel3 = Math.toDegrees(Math.atan2((y_left + Math.sin(corner3) * OMEGA), (x_left + Math.cos(corner3) * OMEGA)));

        double speed0 = Math.sqrt(Math.pow(y_left + Math.sin(corner0) * OMEGA, 2) + Math.pow(x_left + Math.cos(corner0) * OMEGA, 2));
        double speed1 = Math.sqrt(Math.pow(y_left + Math.sin(corner1) * OMEGA, 2) + Math.pow(x_left + Math.cos(corner1) * OMEGA, 2));
        double speed2 = Math.sqrt(Math.pow(y_left + Math.sin(corner2) * OMEGA, 2) + Math.pow(x_left + Math.cos(corner2) * OMEGA, 2));
        double speed3 = Math.sqrt(Math.pow(y_left + Math.sin(corner3) * OMEGA, 2) + Math.pow(x_left + Math.cos(corner3) * OMEGA, 2));
        double max = speed0;

        if (speed1 > max) {
            max = speed1;
        }
        if (speed2 > max) {
            max = speed2;
        }
        if (speed3 > max) {
            max = speed3;
        }
        if (max > ROBOT_MAX_SPEED) {
            speed0 = Range.scale(speed0, 0, max, 0, ROBOT_MAX_SPEED);
            speed1 = Range.scale(speed1, 0, max, 0, ROBOT_MAX_SPEED);
            speed2 = Range.scale(speed2, 0, max, 0, ROBOT_MAX_SPEED);
            speed3 = Range.scale(speed3, 0, max, 0, ROBOT_MAX_SPEED);
        }

        if (ROBOT_STATUS != RELEASE) {
            linearOpMode.telemetry.addData("DELTA:", delta);
            linearOpMode.telemetry.addData("OMEGA:", OMEGA);
            linearOpMode.telemetry.addData("STRAFE SPEED:", STR);
            linearOpMode.telemetry.addData("STRAFE ANGLE:", STR_ANGLE);
        }

        swivel(swivel0, swivel1, swivel2, swivel3);
        setPower(speed0, speed1, speed2, speed3);
    }

    private void turnDrive(double strafe_angle, double strafe_power, double turn_power, double yaw) {

        double OMEGA = turn_power; // Rotational speed: Clockwise is positive and Anti-Clockwise is negative
        double STR = strafe_power; // Strafing speed
        double STR_ANGLE = strafe_angle; // Strafing angle
        double corner0 = Math.atan2(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2);
        double corner1 = corner0 - (HALF_PI);
        double corner2 = corner1 - (HALF_PI);
        double corner3 = corner2 - (HALF_PI);

        /* Make angles field-oriented */
        STR_ANGLE -= yaw;
        double abscissa = STR * Math.cos(Math.toRadians(STR_ANGLE)); // TODO: generalize gyro angles so they don't depend on the hub's orientation
        double ordinate = STR * Math.sin(Math.toRadians(STR_ANGLE));

        double swivel0 = Math.toDegrees(Math.atan2((ordinate + Math.sin(corner0) * OMEGA), (abscissa + Math.cos(corner0) * OMEGA)));
        double swivel1 = Math.toDegrees(Math.atan2((ordinate + Math.sin(corner1) * OMEGA), (abscissa + Math.cos(corner1) * OMEGA)));
        double swivel2 = Math.toDegrees(Math.atan2((ordinate + Math.sin(corner2) * OMEGA), (abscissa + Math.cos(corner2) * OMEGA)));
        double swivel3 = Math.toDegrees(Math.atan2((ordinate + Math.sin(corner3) * OMEGA), (abscissa + Math.cos(corner3) * OMEGA)));

        double speed0 = Math.sqrt(Math.pow(ordinate + Math.sin(corner0) * OMEGA, 2) + Math.pow(abscissa + Math.cos(corner0) * OMEGA, 2));
        double speed1 = Math.sqrt(Math.pow(ordinate + Math.sin(corner1) * OMEGA, 2) + Math.pow(abscissa + Math.cos(corner1) * OMEGA, 2));
        double speed2 = Math.sqrt(Math.pow(ordinate + Math.sin(corner2) * OMEGA, 2) + Math.pow(abscissa + Math.cos(corner2) * OMEGA, 2));
        double speed3 = Math.sqrt(Math.pow(ordinate + Math.sin(corner3) * OMEGA, 2) + Math.pow(abscissa + Math.cos(corner3) * OMEGA, 2));
        double max = speed0;

        if (speed1 > max) {
            max = speed1;
        }
        if (speed2 > max) {
            max = speed2;
        }
        if (speed3 > max) {
            max = speed3;
        }
        if (max > ROBOT_MAX_SPEED) {
            speed0 = Range.scale(speed0, 0, max, 0, ROBOT_MAX_SPEED);
            speed1 = Range.scale(speed1, 0, max, 0, ROBOT_MAX_SPEED);
            speed2 = Range.scale(speed2, 0, max, 0, ROBOT_MAX_SPEED);
            speed3 = Range.scale(speed3, 0, max, 0, ROBOT_MAX_SPEED);
        }

        swivel(swivel0, swivel1, swivel2, swivel3);
        setPower(speed0, speed1, speed2, speed3);
    }

    public void swivel(double toAngle) {
        module0.swivel(toAngle);
        module1.swivel(toAngle);
        module2.swivel(toAngle);
        module3.swivel(toAngle);


        /*
        double[] distances = new double[4];
        distances[0] = module0.swivel(toAngle);
        distances[1] = module1.swivel(toAngle);
        distances[2] = module2.swivel(toAngle);
        distances[3] = module3.swivel(toAngle);
        Arrays.sort(distances);

        long wait = Math.round(servoSpeed * distances[0] * 1000);

        if (wait >= CYCLE_MS * 2) {
            setPower(0);
            linearOpMode.sleep(wait);
        }

        if (!ROBOT_STATUS.equals(RELEASE))
            linearOpMode.telemetry.addLine("Sleep: " + wait);
        */
    }

    public void swivel(double angle0, double angle1, double angle2, double angle3) {
        module0.swivel(angle0);
        module1.swivel(angle1);
        module2.swivel(angle2);
        module3.swivel(angle3);

        /*
        double[] distances = new double[4];
        distances[0] = module0.swivel(angle0);
        distances[1] = module1.swivel(angle1);
        distances[2] = module2.swivel(angle2);
        distances[3] = module3.swivel(angle3);
        Arrays.sort(distances);

        long wait = Math.round(servoSpeed * distances[0] * 1000);

        if (wait >= CYCLE_MS * 2) {
            setPower(0);
            linearOpMode.sleep(wait);
        }

        if (!ROBOT_STATUS.equals(RELEASE))
            linearOpMode.telemetry.addLine("Sleep: " + wait);
        */
    }

    public void setPower(double power) {
        power = Range.clip(power, 0, ROBOT_MAX_SPEED);
        module0.setPower(power);
        module1.setPower(power);
        module2.setPower(power);
        module3.setPower(power);
    }

    public void setPower(double power0, double power1, double power2, double power3) {
        power0 = Range.clip(power0, 0, ROBOT_MAX_SPEED);
        power1 = Range.clip(power1, 0, ROBOT_MAX_SPEED);
        power2 = Range.clip(power2, 0, ROBOT_MAX_SPEED);
        power3 = Range.clip(power3, 0, ROBOT_MAX_SPEED);

        module0.setPower(power0);
        module1.setPower(power1);
        module2.setPower(power2);
        module3.setPower(power3);
    }

    public void setPath(Path _path) {
        path = _path;
        trackingPoint = new Point(0, 0, path.heading(0));
        splineSegment = 1;
    }

    /*
     * 1. Update robot pose
     * 2. Find robot location on path
     * 3. Steer to close the gap - this includes all the math
     */
    public void stanleyPursuit() {

        if (swerveState == PATH_FOLLOWING_COMPLETE ||
                swerveState == SwerveState.HUMAN_INPUT) {
            return;
        }

        // Update robot pose
        double yaw = roundTo2DecimalPlaces(swerveKinematics.getYaw());
        double velocity = roundTo2DecimalPlaces(swerveKinematics.getVelocity());
        double roundedComX = roundTo2DecimalPlaces(swerveKinematics.getCenterOfMass().getX());
        double roundedComY = roundTo2DecimalPlaces(swerveKinematics.getCenterOfMass().getY());

        CoM.setCoordinates(roundedComX, roundedComY);

        // Find robot location on path
        double trace = path.findTrackingPoint(trackingPoint, CoM);
        if (Double.isNaN(trace) || trace >= path.spline.getKnots()[path.spline.getN()]) {
            swerveState = PATH_FOLLOWING_COMPLETE;
            setPower(0);
            linearOpMode.telemetry.addLine(Double.toString(trace));
        } else {
            trackingPoint.setCoordinates(trace, path.spline.value(trace));
            trackingPoint.setHeading(path.heading(trackingPoint.getX()));


            // Calculate cross-track error
            double distance = Math.hypot(trackingPoint.getX() - CoM.getX(), trackingPoint.getY() - CoM.getY());
            distance = Math.copySign(distance, trackingPoint.getY() - CoM.getY());
            double crossTrackAngle = Math.toDegrees(Math.atan2(distance, path.getDirection().assignDirection(velocity / kS)));

            turnDrive(crossTrackAngle, .05,0, yaw);


            if (!ROBOT_STATUS.equals(RELEASE)) {
                debugger.addData("Yaw", Double.toString(yaw));
                debugger.addData(PATH.toString(), path.toString());
                debugger.addData(PX.toString(), Double.toString(trackingPoint.getX()));
                debugger.addData(PY.toString(), Double.toString(trackingPoint.getY()));
                debugger.addData(ERROR.toString(), Double.toString(distance));
                debugger.addData(CORRECTION.toString(), Double.toString(crossTrackAngle));
                debugger.addData(VELOCITY.toString(), Double.toString(velocity));
                debugger.addData(PATH.toString(), path.toString());
                debugger.addData(WA.toString(), Double.toString(module0.getServoPosition()));
                debugger.addData(VELOCITY.toString(), Double.toString(velocity));
            }
        }
    }
}
