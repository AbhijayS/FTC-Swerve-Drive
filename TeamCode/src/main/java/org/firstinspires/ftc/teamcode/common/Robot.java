package org.firstinspires.ftc.teamcode.common;

import android.util.DebugUtils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Path;
import org.firstinspires.ftc.teamcode.common.utilities.WayPoint;
import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;
import org.firstinspires.ftc.teamcode.modules.elevator.LiftModule;
import org.firstinspires.ftc.teamcode.modules.jewelswatter.JewelSwatter;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;

public class Robot {
    WayPoint[] wayPoints;
    Path path;
    Clamp clamp;
    LiftModule lift;
    JewelSwatter jewelSwatter;
    SwerveDrive swerveDrive;

    public Robot(LinearOpMode linearOpMode) {
        clamp = new Clamp(linearOpMode);
        lift = new LiftModule(linearOpMode);
        jewelSwatter = new JewelSwatter(linearOpMode.hardwareMap);
        swerveDrive = new SwerveDrive(linearOpMode, null);
    }

    public Robot(LinearOpMode linearOpMode, Debugger debugger) {
        clamp = new Clamp(linearOpMode);
        lift = new LiftModule(linearOpMode);
        jewelSwatter = new JewelSwatter(linearOpMode.hardwareMap);
        swerveDrive = new SwerveDrive(linearOpMode, debugger);
    }
}
