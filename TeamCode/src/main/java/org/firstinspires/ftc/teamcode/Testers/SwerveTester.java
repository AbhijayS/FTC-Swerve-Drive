package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotModules.Path;
import org.firstinspires.ftc.teamcode.RobotModules.SwerveDrive;
import org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants;

public class SwerveTester extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        SwerveDrive swerveDrive = new SwerveDrive(this);

        double x[] = {
            0,10
        };
        double y[] = {
                0,0
        };
        Path toInterpolate = new Path(x, y);
        swerveDrive.setPath(toInterpolate);
        swerveDrive.requestState(UniversalConstants.SwerveState.PATH_FOLLOWING);

        waitForStart();

        while (swerveDrive.returnSwerveState().equals(UniversalConstants.SwerveState.PATH_FOLLOWING)){
            swerveDrive.swerveKinematics.update();
            swerveDrive.stanleyPursuit();
        }


    }
}
