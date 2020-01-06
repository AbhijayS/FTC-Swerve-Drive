package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.states.SwerveState;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Direction;
import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;
import org.firstinspires.ftc.teamcode.common.utilities.Path;
import org.firstinspires.ftc.teamcode.common.utilities.Stopwatch;
import org.firstinspires.ftc.teamcode.common.utilities.WayPoint;
import org.firstinspires.ftc.teamcode.modules.HardStops;
import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;
import org.firstinspires.ftc.teamcode.modules.elevator.Lift;
import org.firstinspires.ftc.teamcode.modules.jewelswatter.JewelSwatter;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;

public class Robot {

    // various hardware variables
    private JewelSwatter jewelSwatter;
    private Clamp clamp;
    private Lift lift;

    // path following variables
    private Path path;
    private WayPoint[] wayPoints;
    private WayPoint currentWaypoint; // current waypoint
    private boolean pidOverride; // PID target position used only when PID is enabled

    // private variables
    private SwerveDrive swerveDrive;
    private Telemetry telemetry;
    private Debugger debugger;
    private Stopwatch stopwatch;
    private RobotState robotState;
    private Gamepad gamepad;
    private HardStops hardStops;

    public enum RobotState {
        FIELD_ORIENTED, // default
        ROBOT_ORIENTED,
        PATH_FOLLOWING
    }

    public Robot(LinearOpMode linearOpMode, Debugger debugger) {
        this.telemetry = linearOpMode.telemetry;
        this.debugger = debugger;
        this.stopwatch = new Stopwatch();
        this.pidOverride = false;
        this.robotState = RobotState.FIELD_ORIENTED;

        this.lift = new Lift(linearOpMode.hardwareMap, debugger);
        this.clamp = new Clamp(linearOpMode.hardwareMap);
        this.jewelSwatter = new JewelSwatter(linearOpMode.hardwareMap,debugger);
        this.swerveDrive = new SwerveDrive(linearOpMode, debugger);
        this.gamepad = new Gamepad(linearOpMode);
        this.hardStops = new HardStops(linearOpMode.hardwareMap);
    }

    public void setWayPoints(WayPoint[] wayPoints) {
        // make sure paths can be set internally
        this.path = new Path(debugger, wayPoints);
        swerveDrive.requestState(SwerveState.PATH_FOLLOWING);
        swerveDrive.setPath(path, 0.1);

        // update private variables
        this.wayPoints = wayPoints;
        this.currentWaypoint = path.SEGMENT_START;
    }

    public void requestState(RobotState newState) {
        if (newState != robotState) {
            switch (newState) {
                case PATH_FOLLOWING: {
                    swerveDrive.requestState(SwerveState.PATH_FOLLOWING);
                    swerveDrive.brakesOn();
                    robotState = newState;
                    break;
                }

                default:
                    this.path = null;
                    this.wayPoints = null;
                    this.currentWaypoint = null;
                    this.hardStops.requestState(HardStops.HardStopState.STOW_ALL);
                    swerveDrive.requestState(SwerveState.HUMAN_INPUT);
                    swerveDrive.brakesOff();
                    robotState = newState;
                    break;
            }
        }
    }

    public void updateAll() {
        switch (robotState) {
            case PATH_FOLLOWING: {
                if (wayPoints == null)
                    throw new RuntimeException("Way Points have not been set up yet!");
                parallelManeuvers();
                if (!currentWaypoint.equals(path.SEGMENT_START)) {
                    stopwatch.reset();
                    pidOverride = false;
                    swerveDrive.disablePID();
                    currentWaypoint = path.SEGMENT_START;
                }
                swerveDrive.swerveKinematics.update();
                jewelSwatter.update();
                clamp.update();
                hardStops.update();
                lift.update();
                debugger.log();
                break;
            }

            case ROBOT_ORIENTED: {
                telemetry.addLine(clamp.getStatus());
                telemetry.addLine(lift.getStatus());
                telemetry.addLine(hardStops.getStatus());
                swerveDrive.fod(gamepad);
                clamp.updateByGamepad(gamepad);
                lift.updateTargetLevelUsingGamepad(gamepad);
                hardStops.updateUsingGamepad(gamepad);
                gamepad.update();
                debugger.log();
                break;
            }

            default: {
                telemetry.addLine(clamp.getStatus());
                telemetry.addLine(lift.getStatus());
                telemetry.addLine(hardStops.getStatus());
                swerveDrive.fod(gamepad);
                clamp.updateByGamepad(gamepad);
                lift.updateTargetLevelUsingGamepad(gamepad);
                hardStops.updateUsingGamepad(gamepad);
                swerveDrive.swerveKinematics.update();
                gamepad.update();
                debugger.log();
                break;
            }
        }
    }

    private void parallelManeuvers() {
        if (stopwatch.millis() >= currentWaypoint.WAIT_MILLIS) {
            // continue movement
            if (currentWaypoint.enablePID) {
                if (path.DIRECTION == Direction.FORWARD || path.DIRECTION == Direction.REVERSE) {
                    if (!pidOverride) {
                        if (path.DIRECTION == Direction.FORWARD)
                            swerveDrive.swivel(90);
                        else
                            swerveDrive.swivel(-90);
                        try {
                            Thread.sleep(100);
                        } catch (Exception e) {
                            throw new RuntimeException(e.getMessage());
                        }
                        double distance = path.SEGMENT_END.Y - swerveDrive.swerveKinematics.getCenterOfMass().getY();
                        swerveDrive.movePID(distance, currentWaypoint.POWER);
                        swerveDrive.enablePID();
                        pidOverride = true;
                    }
                }
                path.pathFollowing(swerveDrive.swerveKinematics.getCenterOfMass());
            } else {
                UniversalConstants.kS = currentWaypoint.kS;
                swerveDrive.setPivotX(currentWaypoint.PIVOT_X);
                swerveDrive.setPivotY(currentWaypoint.PIVOT_Y);
                swerveDrive.setMaxPower(currentWaypoint.POWER);
                swerveDrive.stanleyPursuit();
            }
        } else { // still waiting
            if (!stopwatch.isRunning())
                stopwatch.start();
            // stop dt movement
            swerveDrive.fod(90,0,swerveDrive.turnPID(currentWaypoint.Z),swerveDrive.swerveKinematics.getYaw());
        }
        jewelSwatter.requestState(currentWaypoint.JEWEL_SWATTER_STATE);
        clamp.requestState(currentWaypoint.CLAMP_STATE);
        hardStops.requestState(currentWaypoint.HARD_STOP_STATE);
        lift.setTargetLevel(currentWaypoint.liftHeight);
        lift.requestState(currentWaypoint.liftState);

        // lift stuff
    }
}
