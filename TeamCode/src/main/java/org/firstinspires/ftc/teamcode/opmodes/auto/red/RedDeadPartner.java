package org.firstinspires.ftc.teamcode.opmodes.auto.red;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.states.SwerveState;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Path;
import org.firstinspires.ftc.teamcode.common.utilities.Util;
import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;
import org.firstinspires.ftc.teamcode.modules.jewelswatter.JewelSwatter;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.roundTo2DecimalPlaces;

@TeleOp(name = "TestOp: Red Dead Partner")
public class RedDeadPartner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) UniversalConstants.Debugging.getDebuggingMarkers());
        robotDebugger.initialize("Auto_Red_Dead_Partner");

        SwerveDrive swerveDrive = new SwerveDrive(this, robotDebugger);
        Clamp clamp = new Clamp(this);
        JewelSwatter jewelSwatter = new JewelSwatter(this.hardwareMap);

        double[] x = {0, 0, 0};
        double[] y = {0, -12.5, -22};
        double[] z = {90, 90, 90};

        Path interpolant = new Path(this,robotDebugger, x, y, z);
        swerveDrive.setPath(interpolant,1);
        swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

        String stonePosition = "C";

        telemetry.update();
        waitForStart();
        switch (stonePosition) {

            case("A"):
            //Moves towards quarry
            while (opModeIsActive()) {
                if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                    break;
                }
                //Opens left clamp
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.DOCK_LEFT);
                swerveDrive.swerveKinematics.update();
                swerveDrive.stanleyPursuit();
                robotDebugger.log();
            }

            // Clamps the sky stone
            clamp.clamp();
            jewelSwatter.requestState(JewelSwatter.JewelSwatterState.POSSESS_LEFT);
            Thread.sleep(500);

            x = new double[]{0, 0, 0};
            y = new double[]{-22, -21, -18};
            z = new double[]{90, 90, 90};

            interpolant = new Path(this, robotDebugger, x, y, z);
            swerveDrive.setPath(interpolant, 1);
            swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

            //Backs up
            while (opModeIsActive()) {
                if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                    break;
                }
                //Raises sky stone
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT);
                swerveDrive.swerveKinematics.update();
                swerveDrive.stanleyPursuit();
                robotDebugger.log();
            }

            clamp.coast();

            x = new double[]{0, 25, 47, 47};
            y = new double[]{-18, -16, -16, -18};
            z = new double[]{90, 90, 90, 90};

            interpolant = new Path(this, robotDebugger, x, y, z);
            swerveDrive.setPath(interpolant, 1);
            swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

            //Moves across field to foundation
            while (opModeIsActive()) {
                if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                    break;
                }
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.POSSESS_LEFT);
                swerveDrive.swerveKinematics.update();
                swerveDrive.stanleyPursuit();
                robotDebugger.log();
            }

            //Releases sky stone
            jewelSwatter.requestState(JewelSwatter.JewelSwatterState.DOCK_LEFT);
            Thread.sleep(500);

            x = new double[]{47, 47, 25, 0};
            y = new double[]{-18, -16, -16, -18};
            z = new double[]{90, 90, 90, 90};

            interpolant = new Path(this, robotDebugger, x, y, z);
            swerveDrive.setPath(interpolant, 1);
            swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

            //Returns across field to 2nd sky stone
            while (opModeIsActive()) {
                if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                    break;
                }
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.STOW_ALL);
                swerveDrive.swerveKinematics.update();
                swerveDrive.stanleyPursuit();
                robotDebugger.log();
            }

            x = new double[]{0, -10, -22};
            y = new double[]{-18, -18, -25};
            z = new double[]{90, 90, 90};

            interpolant = new Path(this, robotDebugger, x, y, z);
            swerveDrive.setPath(interpolant, 1);
            swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

            //Grabs second sky stone
            while (opModeIsActive()) {
                if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                    break;
                }
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.DOCK_LEFT);
                swerveDrive.swerveKinematics.update();
                swerveDrive.stanleyPursuit();
                robotDebugger.log();
            }

            jewelSwatter.requestState(JewelSwatter.JewelSwatterState.POSSESS_LEFT);
            Thread.sleep(500);

            x = new double[]{-22, -15, 0};
            y = new double[]{-31, -28, -22};
            z = new double[]{90, 90, 180};

            interpolant = new Path(this, robotDebugger, x, y, z);
            swerveDrive.setPath(interpolant, 1);
            swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

            //Backs up from second sky stone
            while (opModeIsActive()) {
                if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                    break;
                }
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.RIDE_HIGH_LEFT);
                swerveDrive.swerveKinematics.update();
                swerveDrive.stanleyPursuit();
                robotDebugger.log();
            }


            // ride low

            x = new double[]{0, 25, 50};
            y = new double[]{-22, -22, -29};
            z = new double[]{180, 180, 220};

            interpolant = new Path(this, robotDebugger, x, y, z);
            swerveDrive.setPath(interpolant, 1);
            swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

            //Positions in front of 2nd sky stone
            while (opModeIsActive()) {
                if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                    break;
                }
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT);
                swerveDrive.swerveKinematics.update();
                swerveDrive.stanleyPursuit();
                robotDebugger.log();
            }

            // park

            jewelSwatter.requestState(JewelSwatter.JewelSwatterState.DOCK_LEFT);
            Thread.sleep(500);

            x = new double[]{50, 47, 45};
            y = new double[]{-29, -29, -29};
            z = new double[]{180, 180, 180};

            interpolant = new Path(this, robotDebugger, x, y, z);
            swerveDrive.setPath(interpolant, 1);
            swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

            while (opModeIsActive()) {
                if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                    break;
                }
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.STOW_ALL);
                swerveDrive.swerveKinematics.update();
                swerveDrive.stanleyPursuit();
                robotDebugger.log();
            }

            while (opModeIsActive()) {

            }

            robotDebugger.stopLogging();




            //CASE C
            case "C":
                //Moves towards quarry
                while (opModeIsActive()) {
                    if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                        break;
                    }
                    //Opens left clamp
                    jewelSwatter.requestState(JewelSwatter.JewelSwatterState.DOCK_RIGHT);
                    swerveDrive.swerveKinematics.update();
                    swerveDrive.stanleyPursuit();
                    robotDebugger.log();
                }

                // Clamps the sky stone
                clamp.clamp();
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.POSSESS_RIGHT);
                Thread.sleep(500);

                x = new double[]{0, 0, 0};
                y = new double[]{-22, -21, -16};
                z = new double[]{90, 90, 90};

                interpolant = new Path(this, robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, 1);
                swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

                //Backs up
                while (opModeIsActive()) {
                    if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                        break;
                    }
                    //Raises sky stone
                    jewelSwatter.requestState(JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT);
                    swerveDrive.swerveKinematics.update();
                    swerveDrive.stanleyPursuit();
                    robotDebugger.log();
                }

                clamp.coast();

                x = new double[]{0, 25, 47, 47};
                y = new double[]{-16, -14, -14, -16};
                z = new double[]{90, 90, 90, 90};

                interpolant = new Path(this, robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, 1);
                swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

                //Moves across field to foundation
                while (opModeIsActive()) {
                    if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                        break;
                    }
                    jewelSwatter.requestState(JewelSwatter.JewelSwatterState.POSSESS_RIGHT);
                    swerveDrive.swerveKinematics.update();
                    swerveDrive.stanleyPursuit();
                    robotDebugger.log();
                }

                //Releases sky stone
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.DOCK_RIGHT);
                Thread.sleep(500);

                x = new double[]{47, 47, 25, 0};
                y = new double[]{-16, -14, -16, -16};
                z = new double[]{90, 90, 90, 90};

                interpolant = new Path(this, robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, 1);
                swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

                //Returns across field to 2nd sky stone
                while (opModeIsActive()) {
                    if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                        break;
                    }
                    jewelSwatter.requestState(JewelSwatter.JewelSwatterState.STOW_ALL);
                    swerveDrive.swerveKinematics.update();
                    swerveDrive.stanleyPursuit();
                    robotDebugger.log();
                }

                x = new double[]{0, -10, -22};
                y = new double[]{-16, -18, -25};
                z = new double[]{90, 90, 90};

                interpolant = new Path(this, robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, 1);
                swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

                //Grabs second sky stone
                while (opModeIsActive()) {
                    if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                        break;
                    }
                    jewelSwatter.requestState(JewelSwatter.JewelSwatterState.DOCK_RIGHT);
                    swerveDrive.swerveKinematics.update();
                    swerveDrive.stanleyPursuit();
                    robotDebugger.log();
                }

                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.POSSESS_RIGHT);
                Thread.sleep(500);

                x = new double[]{-22, -15, 0};
                y = new double[]{-31, -28, -22};
                z = new double[]{90, 90, 180};

                interpolant = new Path(this, robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, 1);
                swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

                //Backs up from second sky stone
                while (opModeIsActive()) {
                    if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                        break;
                    }
                    jewelSwatter.requestState(JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT);
                    swerveDrive.swerveKinematics.update();
                    swerveDrive.stanleyPursuit();
                    robotDebugger.log();
                }


                // ride low

                x = new double[]{0, 25, 50};
                y = new double[]{-22, -22, -29};
                z = new double[]{180, 180, 220};

                interpolant = new Path(this, robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, 1);
                swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

                //Positions in front of 2nd sky stone
                while (opModeIsActive()) {
                    if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                        break;
                    }
                    jewelSwatter.requestState(JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT);
                    swerveDrive.swerveKinematics.update();
                    swerveDrive.stanleyPursuit();
                    robotDebugger.log();
                }

                // park

                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.DOCK_RIGHT);
                Thread.sleep(500);

                x = new double[]{50, 47, 45};
                y = new double[]{-29, -29, -29};
                z = new double[]{180, 180, 180};

                interpolant = new Path(this, robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, 1);
                swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

                while (opModeIsActive()) {
                    if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                        break;
                    }
                    jewelSwatter.requestState(JewelSwatter.JewelSwatterState.STOW_ALL);
                    swerveDrive.swerveKinematics.update();
                    swerveDrive.stanleyPursuit();
                    robotDebugger.log();
                }

                while (opModeIsActive()) {

                }

                robotDebugger.stopLogging();
        }
    }
}
