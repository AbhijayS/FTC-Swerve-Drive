package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.states.SwerveState;
import org.firstinspires.ftc.teamcode.common.utilities.Path;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;

@Autonomous(name = "Swerve Tester")
public class SwerveTester extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        SwerveDrive swerveDrive = new SwerveDrive(this);

        double x[] = {
                0,.5 * 10, 10,
        };
        double y[] = {
                0,0,0
        };
        Path toInterpolate = new Path(this, x, y);
        swerveDrive.setPath(toInterpolate);
        swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            swerveDrive.swerveKinematics.update();
            swerveDrive.stanleyPursuit();
            telemetry.update();
        }
    }
}