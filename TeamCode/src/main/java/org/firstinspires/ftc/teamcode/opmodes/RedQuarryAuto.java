package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.states.SwerveState;
import org.firstinspires.ftc.teamcode.common.utilities.Path;
import org.firstinspires.ftc.teamcode.common.utilities.Stopwatch;
import org.firstinspires.ftc.teamcode.modules.sensory.TensorFlowLite;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;

@Autonomous(name = "Auto Quarry Side")
public class RedQuarryAuto extends LinearOpMode {

    enum AutoStates {
        CASE_A(new double[]{0, 0, 0}, new double[]{0, 0, 0}),
        CASE_B(new double[]{0, 0, 0}, new double[]{0, 0, 0}),
        CASE_C(new double[]{0, 0, 0}, new double[]{0, 0, 0}),
        CASE_A1(new double[]{0, 0, 0}, new double[]{0, 0, 0}),
        CASE_B1(new double[]{0, 0, 0}, new double[]{0, 0, 0}),
        CASE_C1(new double[]{0, 0, 0}, new double[]{0, 0, 0}),
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
        TensorFlowLite tensorFlowLite = new TensorFlowLite(this, .12);
        //swerveDrive.setPath(toInterpolate);
        swerveDrive.requestState(SwerveState.PATH_FOLLOWING);
        boolean transition = false;
        String pattern = "";

        waitForStart();

        Stopwatch stopwatch = new Stopwatch();
        stopwatch.start();
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
                    if (stopwatch.seconds() >= 3) {
                        autoStates = AutoStates.CASE_A;
                        transition = true;
                    }
                    break;


                case CASE_A:
                    swerveDrive.swerveKinematics.update();
                    swerveDrive.stanleyPursuit();
                    if (!swerveDrive.returnSwerveState().equals(UniversalConstants.SwerveState.PATH_FOLLOWING)) {
                        autoStates = AutoStates.CASE_A1;
                        transition = true;
                    }
                    break;

                case CASE_B:
                    swerveDrive.swerveKinematics.update();
                    swerveDrive.stanleyPursuit();
                    if (!swerveDrive.returnSwerveState().equals(UniversalConstants.SwerveState.PATH_FOLLOWING)) {
                        autoStates = AutoStates.CASE_B1;
                        transition = true;
                    }
                    break;

                case CASE_C:
                    swerveDrive.swerveKinematics.update();
                    swerveDrive.stanleyPursuit();
                    if (!swerveDrive.returnSwerveState().equals(UniversalConstants.SwerveState.PATH_FOLLOWING)) {
                        autoStates = AutoStates.CASE_C1;
                        transition = true;
                    }
                    break;

            }
            if (transition) {
                Path toInterpolate = new Path(this, autoStates.x, autoStates.y);
                swerveDrive.setPath(toInterpolate);
                swerveDrive.requestState(SwerveState.PATH_FOLLOWING);
                transition = false;
            }
        }
    }
}
