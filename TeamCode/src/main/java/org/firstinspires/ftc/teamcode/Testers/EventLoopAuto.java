package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotModules.Path;
import org.firstinspires.ftc.teamcode.RobotModules.SwerveDrive;
import org.firstinspires.ftc.teamcode.RobotModules.TensorFlowLite;
import org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants;

public class EventLoopAuto extends LinearOpMode {

    enum AutoStates {
        CASE_A(new double[]{0, 0, 0}, new double[]{0, 0, 0}),
        CASE_B(new double[]{0, 0, 0}, new double[]{0, 0, 0}),
        CASE_C(new double[]{0, 0, 0}, new double[]{0, 0, 0}),
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
        //Path toInterpolate = new Path(this, AutoStates.PATH_ONE.x, AutoStates.PATH_ONE.y);
        TensorFlowLite tensorFlowLite = new TensorFlowLite(this,.12);
        //swerveDrive.setPath(toInterpolate);
        swerveDrive.requestState(UniversalConstants.SwerveState.PATH_FOLLOWING);
        boolean transition = false;
        String pattern = "";

        waitForStart();

        while (opModeIsActive()) {
            switch (autoStates) {
                case SCANNING:
                    tensorFlowLite.determinePattern();
                    if (!tensorFlowLite.getPattern().equals("Unknown")) {
                        pattern = tensorFlowLite.getPattern();
                        if (pattern.equals("A")) {
                            autoStates = AutoStates.CASE_A;
                        } else if (pattern.equals("B")) {
                            autoStates = AutoStates.CASE_B;
                        } else {
                            autoStates = AutoStates.CASE_C;
                        }
                        transition = true;
                    }
                    break;
                case CASE_A:
                    swerveDrive.swerveKinematics.update();
                    swerveDrive.stanleyPursuit();
                    if (!swerveDrive.returnSwerveState().equals(UniversalConstants.SwerveState.PATH_FOLLOWING)) {
                        autoStates = AutoStates.PATH_ONE;
                    }
                    break;

            }
            if(transition){
                Path toInterpolate = new Path(this, autoStates.x, autoStates.y);
                swerveDrive.setPath(toInterpolate);
                swerveDrive.requestState(UniversalConstants.SwerveState.PATH_FOLLOWING);
            }
        }
    }
}
