package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.states.SwerveState;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Path;
import org.firstinspires.ftc.teamcode.common.utilities.WayPoint;
import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;
import org.firstinspires.ftc.teamcode.modules.elevator.LiftModule;
import org.firstinspires.ftc.teamcode.modules.jewelswatter.JewelSwatter;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;

import java.util.Arrays;
import java.util.LinkedList;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.CYCLE_MS;

public class Robot {
    private LinearOpMode linearOpMode;
    private Debugger debugger;
    private LinkedList<WayPoint> wayPoints;
    private Path path;
    private Clamp clamp;
    private LiftModule lift;
    private JewelSwatter jewelSwatter;
    private SwerveDrive swerveDrive;
    private int size;
    private int point; // way point that we're following
    private int counter; // countdown for if there's a wait command

    public Robot(LinearOpMode linearOpMode, Debugger debugger) {
        this.linearOpMode = linearOpMode;
        this.debugger = debugger;
        clamp = new Clamp(linearOpMode);
        lift = new LiftModule(linearOpMode);
        jewelSwatter = new JewelSwatter(linearOpMode.hardwareMap);
        swerveDrive = new SwerveDrive(linearOpMode, debugger);
        wayPoints = new LinkedList<>();
        point = 0;
    }

    public void setWayPoints(WayPoint[] wayPoints) {
        this.wayPoints.addAll(Arrays.asList(wayPoints));
        this.size = this.wayPoints.size();
        double rx[] = new double[this.size];
        double ry[] = new double[this.size];
        double rz[] = new double[this.size];
        for (int i = 0; i < this.size; i++) {
            rx[i] = this.wayPoints.get(i).x;
            ry[i] = this.wayPoints.get(i).y;
            rz[i] = this.wayPoints.get(i).z;
        }
        path = new Path(debugger,rx,ry,rz);
        swerveDrive.setPath(path,0.3);
        swerveDrive.requestState(SwerveState.PATH_FOLLOWING);
    }

    public void parallelManeuvers() {
        if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE)
            return;

        WayPoint current = wayPoints.get(point);

        if (counter >= 0) {
            if (current.waitMillis != 0) {
                counter = (int)(current.waitMillis/CYCLE_MS);
                return;
            }
            swerveDrive.stanleyPursuit();
        } else {
            swerveDrive.setPower(0);
            jewelSwatter.requestState(current.jewelSwatterState);
            clamp.requestState(current.clampState);
            counter--;
        }
        updateAll();
    }

    public void updateAll() {
        swerveDrive.swerveKinematics.update();
        jewelSwatter.update();
        clamp.update();
        debugger.log();
    }
}
