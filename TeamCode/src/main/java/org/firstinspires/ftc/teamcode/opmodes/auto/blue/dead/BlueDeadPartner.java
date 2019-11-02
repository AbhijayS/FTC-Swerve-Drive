package org.firstinspires.ftc.teamcode.opmodes.auto.blue.dead;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.states.SwerveState;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Path;
import org.firstinspires.ftc.teamcode.common.utilities.Util;
import org.firstinspires.ftc.teamcode.modules.jewelswatter.JewelSwatter;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;

import java.util.ArrayList;

public class BlueDeadPartner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) UniversalConstants.Debugging.getDebuggingMarkers());
        robotDebugger.initialize("Auto_Blue_Dead_Partner");

        SwerveDrive swerveDrive = new SwerveDrive(this, null);
        JewelSwatter jewelSwatter = new JewelSwatter(this);
        jewelSwatter.stowAll();

        double x[] = {0,0,0};
        double y[] = {0,0,0};
        double z[] = {90,90,90};

        Path interpolant = new Path(this, x, y, z);
        swerveDrive.setPath(interpolant);
        swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

        int counter = 0;

        telemetry.update();
        waitForStart();

        // Approaching the quarry
        while (opModeIsActive()) {
            if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                break;
            }
            jewelSwatter.preclampLeft();
            swerveDrive.swerveKinematics.update();
            swerveDrive.stanleyPursuit();
            robotDebugger.log();
        }

        // Clamp the stone
        while (opModeIsActive() && counter < 3) {
            jewelSwatter.clampLeft();
            counter++;
        }
        counter = 0;

        robotDebugger.stopLogging();
    }
}
