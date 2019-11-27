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
import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;
import org.firstinspires.ftc.teamcode.modules.elevator.LiftModule;
import org.firstinspires.ftc.teamcode.modules.jewelswatter.JewelSwatter;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;

public class Robot {

    // various hardware variables
    private JewelSwatter jewelSwatter;
    private Clamp clamp;
    private LiftModule lift;

    // path following variables
    private Path path;
    private WayPoint[] wayPoints;
    private WayPoint wayPoint; // current waypoint
    private boolean pidOverride; // PID target position used only when PID is enabled

    // private variables
    private SwerveDrive swerveDrive;
    private Telemetry telemetry;
    private Debugger debugger;
    private int size;
    private Stopwatch stopwatch;
    private RobotState robotState;
    private Gamepad gamepad;

    public enum RobotState {
        PATH_FOLLOWING, HUMAN_OPERATOR
    }

    public Robot(LinearOpMode linearOpMode, Debugger debugger) {
        this.telemetry = linearOpMode.telemetry;
        this.debugger = debugger;
        this.clamp = new Clamp(linearOpMode);
        this.lift = new LiftModule(linearOpMode);
        this.jewelSwatter = new JewelSwatter(linearOpMode.hardwareMap);
        this.swerveDrive = new SwerveDrive(linearOpMode, debugger);
        this.stopwatch = new Stopwatch();
        this.pidOverride = false;
        this.robotState = null;
        this.gamepad = new Gamepad(linearOpMode);
    }

    public void setWayPoints(WayPoint[] wayPoints) {
        this.wayPoints = wayPoints;
        this.size = wayPoints.length;
        this.path = new Path(debugger, wayPoints);
        swerveDrive.setPath(path, 0.25);
        swerveDrive.requestState(SwerveState.PATH_FOLLOWING);
        this.wayPoint = path.CURRENT_WAYPOINT;
    }

    public void requestState(RobotState newState) {
        if (newState != robotState) {
            if (newState == RobotState.HUMAN_OPERATOR)
                swerveDrive.requestState(SwerveState.HUMAN_INPUT);
            else
                swerveDrive.requestState(SwerveState.PATH_FOLLOWING);
            robotState = newState;
        }
    }

    public void updateAll() {
        if (robotState == RobotState.HUMAN_OPERATOR)
            swerveDrive.fod(gamepad);
        else if (robotState == RobotState.PATH_FOLLOWING){
            parallelManeuvers();
            if (!wayPoint.equals(path.CURRENT_WAYPOINT)) {
                stopwatch.reset();
                pidOverride = false;
                swerveDrive.disablePID();
                wayPoint = path.CURRENT_WAYPOINT;
                telemetry.addLine("NOT SAME");
            }
        }
        swerveDrive.swerveKinematics.update();
        jewelSwatter.update();
        clamp.update();
        debugger.log();
    }

    private void parallelManeuvers() {
        if (stopwatch.millis() >= wayPoint.WAIT_MILLIS) {
            // continue movement
            if (wayPoint.enablePID) {
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
                        double distance = path.SEGMENT_END.Y - path.SEGMENT_START.Y;
                        swerveDrive.movePID(distance, wayPoint.POWER);
                        swerveDrive.enablePID();
                        pidOverride = true;
                    }
                }
                telemetry.addLine("Waiting for PID");
                telemetry.addData("Start", path.SEGMENT_START.Y);
                telemetry.addData("End", path.SEGMENT_END.Y);
                path.pathFollowing(swerveDrive.swerveKinematics.getCenterOfMass());
            } else {
                swerveDrive.stanleyPursuit();
            }
        } else { // still waiting
            if (!stopwatch.isRunning())
                stopwatch.start();
            // stop dt movement
            swerveDrive.setPower(0);
        }

        jewelSwatter.requestState(wayPoint.JEWEL_SWATTER_STATE);
        clamp.requestState(wayPoint.CLAMP_STATE);
        // lift stuff
    }
}
