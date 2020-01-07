package org.firstinspires.ftc.teamcode.modules.swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.states.MotionState;
import org.firstinspires.ftc.teamcode.common.states.SwerveState;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Direction;
import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;
import org.firstinspires.ftc.teamcode.common.utilities.Path;
import org.firstinspires.ftc.teamcode.common.utilities.Pose;

import static org.firstinspires.ftc.teamcode.common.utilities.Debugger.Marker.PATH;
import static org.firstinspires.ftc.teamcode.common.utilities.Debugger.Marker.PX;
import static org.firstinspires.ftc.teamcode.common.utilities.Debugger.Marker.PY;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.HALF_PI;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.ModuleConfig;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.AUTO_MAX_SPEED;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.ROBOT_STATUS;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.Status.RELEASE;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.TELEOP_MAX_SPEED;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.clipAngle;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.kI;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.kP;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.kS;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.roundTo2DecimalPlaces;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.tolerance;
import static org.firstinspires.ftc.teamcode.common.states.SwerveState.HUMAN_INPUT;
import static org.firstinspires.ftc.teamcode.common.states.SwerveState.PATH_FOLLOWING;
import static org.firstinspires.ftc.teamcode.common.states.SwerveState.PATH_FOLLOWING_COMPLETE;

public class SwerveDrive {

    /* Private OpMode members. */
    public SwerveModule module0;
    public SwerveModule module1;
    public SwerveModule module2;
    public SwerveModule module3;
    public SwerveKinematics swerveKinematics;
    private LinearOpMode linearOpMode;
    private Path path = null;
    private double IMU_ZERO;
    private Pose CoM;
    private Pose trackingPose; // Pose on the path to be tracked
    private int splineSegment;
    private Debugger debugger;
    private SwerveState swerveState;
    private MotionState motionState;
    private Pose kinematicsDelta;
    private double headingGoal;
    private boolean headingGoalSet;
    private double maxPower; // path following power
    private double cerr;
    private double lastTargetAngle;
    private double FAKE_COM_X;
    private double FAKE_COM_Y;
    private double kF = 0.09;
    private double kP = 0.1;

    public SwerveDrive(LinearOpMode l, Debugger debugger) {
        linearOpMode = l;
        this.debugger = debugger;
        this.swerveState = HUMAN_INPUT;
        this.motionState = MotionState.STOPPED;

        module0 = new SwerveModule(l.hardwareMap, ModuleConfig.MODULE_ZERO, this);
        module1 = new SwerveModule(l.hardwareMap, ModuleConfig.MODULE_ONE, this);
        module2 = new SwerveModule(l.hardwareMap, ModuleConfig.MODULE_TWO, this);
        module3 = new SwerveModule(l.hardwareMap, ModuleConfig.MODULE_THREE, this);

        CoM = new Pose();
        FAKE_COM_X = 0;
        FAKE_COM_Y = 0;

        this.swerveKinematics = new SwerveKinematics(l, debugger, this);
        this.kinematicsDelta = new Pose(0, 0, 0);
        this.IMU_ZERO = swerveKinematics.getIMU_ZERO();
        this.headingGoal = 90;
        this.headingGoalSet = true;
        this.lastTargetAngle = 90;
        linearOpMode.telemetry.addLine(String.format("Swerve Drive Calibrated in %s status", ROBOT_STATUS));
    }

    public void requestState(SwerveState swerveState) {
        setPower(0);
        this.swerveState = swerveState;
    }

    public SwerveState getState() {
        return swerveState;
    }

    // TODO: Simplify field-oriented drive (fod) code
    public void fod(Gamepad g) {
        if (g.reset_gyro)
            swerveKinematics.resetGyro();

        if (swerveState == PATH_FOLLOWING || swerveState == PATH_FOLLOWING_COMPLETE) {
            return;
        }

        double slow = 1;
        if (g.slowmo)
            slow = 0.25;

        double OMEGA = Range.scale(g.z, 0, 1, 0, TELEOP_MAX_SPEED * slow); // Rotational speed: Clockwise is positive and Anti-Clockwise is negative
        if (Double.compare(OMEGA, 0) == 0) {
            if (!headingGoalSet) {
                headingGoal = Math.rint(swerveKinematics.getCenterOfMass().getDegrees());
                headingGoalSet = true;
            }
            OMEGA = turnPID(headingGoal);
        } else {
            headingGoalSet = false;
        }

        if (g.heading) {
            if (g._45) {
                headingGoal = 45;
            }
            else if (g._135) {
                headingGoal = 135;
            }
            else if (g._225) {
                headingGoal = 225;
            }
            else if (g._315) {
                headingGoal = 315;
            }
            else if (g._0) {
                headingGoal = 0;
            }
            else if (g._90) {
                headingGoal = 90;
            }
            else if (g._180) {
                headingGoal = 180;
            }
            else if (g._270) {
                headingGoal = 270;
            }
            headingGoalSet = true;
        }

//        if (g.x == 0 && g.y == 0) {
//            setPower(0);
//            if (turnPID(headingGoal) != 0) {
//                fod(90,0,turnPID(headingGoal),swerveKinematics.getYaw());
//            }
//            return;
//        }

        double x_left = g.x;
        double y_left = g.y;

        double STR = Range.scale(Math.abs(Math.hypot(x_left, y_left)), 0, 1, 0, TELEOP_MAX_SPEED * slow); // Strafing speed
        double STR_ANGLE = Math.toDegrees(Math.atan2(y_left, x_left)); // Strafing angle
        double corner0 = Math.atan2(ModuleConfig.MODULE_ZERO.rawY()-FAKE_COM_Y, ModuleConfig.MODULE_ZERO.rawX()-FAKE_COM_X) - HALF_PI;
        double corner1 = Math.atan2(ModuleConfig.MODULE_ONE.rawY()-FAKE_COM_Y, ModuleConfig.MODULE_ONE.rawX()-FAKE_COM_X) - HALF_PI;
        double corner2 = Math.atan2(ModuleConfig.MODULE_TWO.rawY()-FAKE_COM_Y, ModuleConfig.MODULE_TWO.rawX()-FAKE_COM_X) - HALF_PI;
        double corner3 = Math.atan2(ModuleConfig.MODULE_THREE.rawY()-FAKE_COM_Y, ModuleConfig.MODULE_THREE.rawX()-FAKE_COM_X) - HALF_PI;

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
        if (max > TELEOP_MAX_SPEED) {
            speed0 = Range.scale(speed0, 0, max, 0, TELEOP_MAX_SPEED * slow);
            speed1 = Range.scale(speed1, 0, max, 0, TELEOP_MAX_SPEED * slow);
            speed2 = Range.scale(speed2, 0, max, 0, TELEOP_MAX_SPEED * slow);
            speed3 = Range.scale(speed3, 0, max, 0, TELEOP_MAX_SPEED * slow);
        }

//        if (ROBOT_STATUS != RELEASE) {
//            linearOpMode.telemetry.addData("DELTA:", delta);
//            linearOpMode.telemetry.addData("OMEGA:", OMEGA);
//            linearOpMode.telemetry.addData("STRAFE SPEED:", STR);
//            linearOpMode.telemetry.addData("STRAFE ANGLE:", STR_ANGLE);
//        }

        setPower(speed0, speed1, speed2, speed3);
        swivel(swivel0, swivel1, swivel2, swivel3);
    }

    public void fod(double strafe_angle, double strafe_power, double turn_power, double yaw) {
        // save computation time if no movement is necessary
        if (strafe_power == 0 && turn_power == 0) {
//            swivel(90);
            setPower(0);
            return;
        }

        double OMEGA = turn_power; // Rotational speed: Clockwise is positive and Anti-Clockwise is negative
        double STR = strafe_power; // Strafing speed
        double STR_ANGLE = strafe_angle; // Strafing angle
        double corner0 = Math.atan2(ModuleConfig.MODULE_ZERO.rawY()-FAKE_COM_Y, ModuleConfig.MODULE_ZERO.rawX()-FAKE_COM_X) - HALF_PI;
        double corner1 = Math.atan2(ModuleConfig.MODULE_ONE.rawY()-FAKE_COM_Y, ModuleConfig.MODULE_ONE.rawX()-FAKE_COM_X) - HALF_PI;
        double corner2 = Math.atan2(ModuleConfig.MODULE_TWO.rawY()-FAKE_COM_Y, ModuleConfig.MODULE_TWO.rawX()-FAKE_COM_X) - HALF_PI;
        double corner3 = Math.atan2(ModuleConfig.MODULE_THREE.rawY()-FAKE_COM_Y, ModuleConfig.MODULE_THREE.rawX()-FAKE_COM_X) - HALF_PI;

        /* Make angles field-oriented */
        STR_ANGLE -= yaw;
        double abscissa = STR * Math.cos(Math.toRadians(STR_ANGLE));
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
        if (max > AUTO_MAX_SPEED) {
            speed0 = Range.scale(speed0, 0, max, 0, AUTO_MAX_SPEED);
            speed1 = Range.scale(speed1, 0, max, 0, AUTO_MAX_SPEED);
            speed2 = Range.scale(speed2, 0, max, 0, AUTO_MAX_SPEED);
            speed3 = Range.scale(speed3, 0, max, 0, AUTO_MAX_SPEED);
        }

        swivel(swivel0, swivel1, swivel2, swivel3);
        setPower(speed0, speed1, speed2, speed3);
    }

    public double turnPID(double targetAngle) {
        if (lastTargetAngle != targetAngle)
            cerr = 0;
        double heading = roundTo2DecimalPlaces(swerveKinematics.getCenterOfMass().getDegrees());
        double setAngle = roundTo2DecimalPlaces(clipAngle(targetAngle));
        double err = roundTo2DecimalPlaces(heading - setAngle);
        err += (err > 180) ? -360 : (err < -180) ? 360 : 0;
        double power;
        if (Math.abs(err) <= tolerance) {
            cerr = 0;
            power = 0;
        }
        else {
            cerr += err;
            power = err * UniversalConstants.kP + cerr * UniversalConstants.kI;
        }
        lastTargetAngle = targetAngle;
        return power;
    }

    public void swivel(double toAngle) {
        module0.swivel(toAngle);
        module1.swivel(toAngle);
        module2.swivel(toAngle);
        module3.swivel(toAngle);


//        double[] distances = new double[4];
//        distances[0] = module0.swivel(toAngle);
//        distances[1] = module1.swivel(toAngle);
//        distances[2] = module2.swivel(toAngle);
//        distances[3] = module3.swivel(toAngle);
//        Arrays.sort(distances);
//
//        long wait = Math.round(servoSpeed * distances[0] * 1000);
//
//        if (wait >= CYCLE_MS * 2) {
//            setPower(0);
//            linearOpMode.sleep(wait);
//        }
//
//        if (!ROBOT_STATUS.equals(RELEASE))
//            linearOpMode.telemetry.addLine("Sleep: " + wait);

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
        power = Range.clip(power, 0, AUTO_MAX_SPEED);
        module0.setPower(power);
        module1.setPower(power);
        module2.setPower(power);
        module3.setPower(power);
    }

    public void setMaxPower(double power) {
        this.maxPower = power;
    }

    public void setPower(double power0, double power1, double power2, double power3) {
        power0 = Range.clip(power0, 0, AUTO_MAX_SPEED);
        power1 = Range.clip(power1, 0, AUTO_MAX_SPEED);
        power2 = Range.clip(power2, 0, AUTO_MAX_SPEED);
        power3 = Range.clip(power3, 0, AUTO_MAX_SPEED);

        module0.setPower(power0);
        module1.setPower(power1);
        module2.setPower(power2);
        module3.setPower(power3);
    }

    public void setPivotX(double inches) {
        FAKE_COM_X = inches;
    }

    public void setPivotY(double inches) {
        FAKE_COM_Y = inches;
    }

    public void setPath(Path path, double maxPower) {
        this.path = path;
        this.CoM.setCoordinates(path.getStartLocation().getX(), path.getStartLocation().getY());
        this.kinematicsDelta.setCoordinates(
                path.getStartLocation().getX() - swerveKinematics.getCenterOfMass().getX(),
                path.getStartLocation().getY() - swerveKinematics.getCenterOfMass().getY()
        );
        this.trackingPose = new Pose(0, 0, path.heading(0));
        this.splineSegment = 1;
        this.maxPower = maxPower;
    }

    /*
     * 1. Update robot pose
     * 2. Find robot location on path
     * 3. Steer to close the gap - this includes all the math
     */
    // TODO: The cross track angle needs to consider the current heading of the robot or implement
    // some kind of PID so that the robot doesn't approach the path too aggressively.
    // Ignoring the current heading of the robot might be leading to the jittery motion.
    public void stanleyPursuit() {

        debugger.addData("Heading Goal", Double.toString(headingGoal));
        double yaw = roundTo2DecimalPlaces(swerveKinematics.getYaw());

        if (swerveState == HUMAN_INPUT) {
            return;
        } else if (swerveState == PATH_FOLLOWING_COMPLETE){
            fod(90,0,turnPID(headingGoal), yaw);
            return;
        }

        // Update robot pose
        double velocity = roundTo2DecimalPlaces(swerveKinematics.getVelocity());
        double roundedComX = roundTo2DecimalPlaces(swerveKinematics.getCenterOfMass().getX() + kinematicsDelta.getX());
        double roundedComY = roundTo2DecimalPlaces(swerveKinematics.getCenterOfMass().getY() + kinematicsDelta.getY());


        CoM.setCoordinates(roundedComX, roundedComY);

        path.pathFollowing(CoM);

        switch (path.PATH_STATE) {
            case END:
                swerveState = PATH_FOLLOWING_COMPLETE;
                headingGoal = trackingPose.getDegrees();
                fod(90,0,turnPID(headingGoal), yaw);
                linearOpMode.telemetry.addLine("Path following done!");
                break;

            case PAUSE:
                setPower(0);
                break;

            case SUSTAIN:
                headingGoal = trackingPose.getDegrees();
                fod(90,0,turnPID(headingGoal), yaw);
                break;

            default:
                // Calculate cross-track error
                trackingPose = path.TRACKING_POSE;
                double distance = Math.hypot(trackingPose.getX() - CoM.getX(), trackingPose.getY() - CoM.getY()) - tolerance;

                double pathAngle = path.heading(trackingPose.getX());
                double crossTrackAngle;

                if (path.DIRECTION == Direction.LEFT || path.DIRECTION == Direction.RIGHT) {
                    distance = Math.copySign(distance, trackingPose.getY() - CoM.getY());
                    crossTrackAngle = Math.toDegrees(Math.atan2(distance, path.DIRECTION.assignDirection(velocity / kS)));
                } else {
                    distance = Math.copySign(distance, trackingPose.getX() - CoM.getX());
                    crossTrackAngle = Math.toDegrees(Math.atan2(path.DIRECTION.assignDirection(velocity / kS), distance));
                }

                double strafeAngle = pathAngle + crossTrackAngle;

                if (!Double.isNaN(trackingPose.getDegrees()))
                    headingGoal = trackingPose.getDegrees();

                debugger.addData("PROFILE", path.getPower());
                fod(strafeAngle, path.getPower(), turnPID(headingGoal), yaw);

                if (!ROBOT_STATUS.equals(RELEASE)) {
//                    debugger.addData(PATH.toString(), path.toString());
//                    debugger.addData(PX.toString(), Double.toString(path.TRACKING_POSE.getX()));
//                    debugger.addData(PY.toString(), Double.toString(path.TRACKING_POSE.getY()));
//                    debugger.addData("HEADING GOAL",Double.toString(headingGoal));
//                    debugger.addData(VELOCITY.toString(), Double.toString(velocity));
                }
        }
    }

    public void enablePID() {
        module0.enablePID();
        module1.enablePID();
        module2.enablePID();
        module3.enablePID();
    }

    public void disablePID() {
        module0.disablePID();
        module1.disablePID();
        module2.disablePID();
        module3.disablePID();
    }

    public void movePID(double position, double power) {
        int encoder = SwerveModule.convertInchesToTicks(position);
        module0.movePID(encoder, power);
        module1.movePID(encoder, power);
        module2.movePID(encoder, power);
        module3.movePID(encoder, power);
    }

    public void resetMotorDirections() {
        module0.resetMotorDirection();
        module1.resetMotorDirection();
        module2.resetMotorDirection();
        module3.resetMotorDirection();
    }

    public void reverseMotorDirections() {
        module0.reverseMotorDirection();
        module1.reverseMotorDirection();
        module2.reverseMotorDirection();
        module3.reverseMotorDirection();
    }

    public void brakesOff() {
        module0.driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        module1.driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        module2.driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        module3.driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void brakesOn() {
        module0.driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        module1.driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        module2.driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        module3.driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
