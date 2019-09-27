package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotModules.Path;
import org.firstinspires.ftc.teamcode.RobotModules.SwerveDrive;
import org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants;

public class EventLoopAuto extends LinearOpMode {

    enum AutoStates {
        PATH_ONE(new double[]{0, 0, 0}, new double[]{0, 0, 0}),
        PATH_TWO(new double[]{0, 0, 0}, new double[]{0, 0, 0}),
        SCANNING(new double[]{0, 0, 0}, new double[]{0, 0, 0});

        public final double[] x;
        public final double[] y;

        AutoStates(double[] x, double[] y) {
            this.x = x;
            this.y = y;
        }

    }



    @Override
    public void runOpMode() throws InterruptedException {
        AutoStates autoStates = AutoStates.SCANNING;
        SwerveDrive swerveDrive = new SwerveDrive(this);
        Path toInterpolate = new Path(AutoStates.PATH_ONE.x, AutoStates.PATH_ONE.y);
        swerveDrive.setPath(toInterpolate);
        swerveDrive.requestState(UniversalConstants.SwerveState.PATH_FOLLOWING);

        waitForStart();

        while (opModeIsActive()) {
            switch (autoStates) {
                case SCANNING:

                    break;
                case PATH_ONE:
                    swerveDrive.swerveKinematics.update();
                    swerveDrive.stanleyPursuit();
                    if(!swerveDrive.returnSwerveState().equals(UniversalConstants.SwerveState.PATH_FOLLOWING)){
                        autoStates = AutoStates.PATH_TWO;
                    }
                    break;

            }
        }
    }
}
