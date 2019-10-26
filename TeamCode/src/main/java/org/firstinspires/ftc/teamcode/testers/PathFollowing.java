package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.utilities.Path;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.utilities.Util;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.states.SwerveState;

import java.util.ArrayList;

@TeleOp(name = "TestOP: Path Following")
public class PathFollowing extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
//        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) UniversalConstants.Debugging.getDebuggingMarkers());
//        robotDebugger.initialize("stanley_kin");

        SwerveDrive swerveDrive = new SwerveDrive(this, null);

        double x[] = {
                0,
                15,
                30,
                45,
                30,
                0
        };
        double y[] = {
                0,
                0,
                0,
                0,
                0,
                0
        };
        double z[] = {
                0,
                90,
                0,
                90,
                45,
                0
        };

        Path toInterpolate = new Path(this, x, y, z);
        swerveDrive.setPath(toInterpolate);
        swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            swerveDrive.swerveKinematics.update();
            swerveDrive.stanleyPursuit();
//            robotDebugger.log();
        }

//        robotDebugger.stopLogging();
    }
}