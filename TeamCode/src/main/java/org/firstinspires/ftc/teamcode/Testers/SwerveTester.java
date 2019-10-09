package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotModules.Path;
import org.firstinspires.ftc.teamcode.RobotModules.SwerveDrive;
import org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants;

@Autonomous(name = "Swerve Tester")
public class SwerveTester extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        SwerveDrive swerveDrive = new SwerveDrive(this);

        double x[] = {
            0,.5 * 100, 100,
        };
        double y[] = {
                0,0,0
        };
        Path toInterpolate = new Path(this, x, y);
        swerveDrive.setPath(toInterpolate);
        swerveDrive.requestState(UniversalConstants.SwerveState.PATH_FOLLOWING);

        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            swerveDrive.swerveKinematics.update();
            swerveDrive.stanleyPursuit();
        }


    }
}
