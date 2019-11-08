package org.firstinspires.ftc.teamcode.opmodes.auto.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.states.SwerveState;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Path;
import org.firstinspires.ftc.teamcode.common.utilities.Util;
import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;
import org.firstinspires.ftc.teamcode.modules.jewelswatter.JewelSwatter;
import org.firstinspires.ftc.teamcode.modules.sensory.TensorFlowLite;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;

import java.util.ArrayList;

@Autonomous(name = "Blue Dead Partner")
public class BlueDeadPartner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) UniversalConstants.Debugging.getDebuggingMarkers());
        robotDebugger.initialize("Auto_Dead_Dead_Partner");

        SwerveDrive swerveDrive = new SwerveDrive(this, robotDebugger);
        Clamp clamp = new Clamp(this);
        JewelSwatter jewelSwatter = new JewelSwatter(this.hardwareMap);
        TensorFlowLite tensorFlowLite = new TensorFlowLite(this,.5);
        tensorFlowLite.activateTfod();

        String stonePosition = "Unknown";
        telemetry.update();

        waitForStart();

        ElapsedTime elapsedTime = new ElapsedTime();
        while(opModeIsActive() && stonePosition.equals("Unknown")){
            tensorFlowLite.twoStone();
            stonePosition = tensorFlowLite.getPattern();
            if (!stonePosition.equals("Unknown") || elapsedTime.time() > 3.0) {
                break;
            }
        }
        tensorFlowLite.shutDownTfod();

        switch (stonePosition) {
            default:
            case "C": {
                double[] x = {0, 0, 0};
                double[] y = {0, -12.5, -22};
                double[] z = {90, 90, 90};

                Path interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, .3);
                swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

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
                clamp.requestState(Clamp.ClampState.CLAMP);
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.POSSESS_LEFT);
                Thread.sleep(500);

                x = new double[]{0, 0, 0};
                y = new double[]{-22, -21, -18};
                z = new double[]{90, 90, 90};

                interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, 0.3);
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

                clamp.requestState(Clamp.ClampState.COAST);

//                x = new double[]{0, 25, 45, 46};
                x = new double[]{0, -25, -44, -45};
                y = new double[]{-18, -19, -19, -19};
                z = new double[]{90, 90, 90, 90};

                interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, 0.3);
                swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

                //Moves across field to foundation
                while (opModeIsActive()) {
                    if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                        break;
                    }
                    jewelSwatter.requestState(JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT);
                    swerveDrive.swerveKinematics.update();
                    swerveDrive.stanleyPursuit();
                    robotDebugger.log();
                }

                //Releases sky stone
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.DOCK_LEFT);
                Thread.sleep(500);

//                x = new double[]{46, 45, 25, 0};
                x = new double[]{-45, -43, -25, 0};
                y = new double[]{-19, -19, -21, -23};
                z = new double[]{90, 90, 90, 90};

                interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, 0.3);
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

                UniversalConstants.kS = 5;
//                x = new double[]{0, -10, -22};
                x = new double[]{0, 10, 18};
                y = new double[]{-23, -26, -29};
                z = new double[]{90, 90, 90};

                interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, 0.3);
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
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.RIDE_HIGH_LEFT);
                Thread.sleep(500);

                UniversalConstants.kS = 0.75;
//                x = new double[]{-22, -15, 0};
                x = new double[]{18, 15, 0};
                y = new double[]{-29, -28, -28};
                z = new double[]{90, 0, 0};

                interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, 0.3);
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
//                x = new double[]{0, 25, 50};
                x = new double[]{0, -25, -50};
                y = new double[]{-28, -29, -30};
                z = new double[]{0, 0, 0};

                interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, 0.3);
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

//                x = new double[]{50, 47, 45};
                x = new double[]{-50, -47, -45};
                y = new double[]{-30, -30, -30};
                z = new double[]{0, 0, 0};

                interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, 0.3);
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

                break;
            }
            case "B": {
                UniversalConstants.kS = 10;
                clamp.requestState(Clamp.ClampState.CLAMP);
                double[] x = {0, 0, 0};
                double[] y = {0, -10, -17};
                double[] z = {90, 90, 90};

                Path interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, .3);
                swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

                //Moves towards quarry
                while (opModeIsActive()) {
                    if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                        break;
                    }
                    //Opens left clamp
                    jewelSwatter.requestState(JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT_OPEN);
                    swerveDrive.swerveKinematics.update();
                    swerveDrive.stanleyPursuit();
                    robotDebugger.log();
                }

                x = new double[]{0, -1.5, -3};
                y = new double[]{-17, -17, -17};
                z = new double[]{90, 90, 90};

                interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, .3);
                swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

                //Moves towards quarry
                while (opModeIsActive()) {
                    if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                        break;
                    }
                    //Opens left clamp
                    jewelSwatter.requestState(JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT_OPEN);
                    swerveDrive.swerveKinematics.update();
                    swerveDrive.stanleyPursuit();
                    robotDebugger.log();
                }

                swerveDrive.swivel(90);
                Thread.sleep(500);
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.DOCK_RIGHT);

                x = new double[]{-3, -3, -3};
                y = new double[]{-17, -18, -19};
                z = new double[]{90, 90, 90};

                interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, .3);
                swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

                //Backs up
                while (opModeIsActive()) {
                    if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                        break;
                    }
                    //Raises sky stone
                    jewelSwatter.requestState(JewelSwatter.JewelSwatterState.DOCK_RIGHT);
                    swerveDrive.swerveKinematics.update();
                    swerveDrive.stanleyPursuit();
                    robotDebugger.log();
                }

                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.POSSESS_RIGHT);
                Thread.sleep(500);

                x = new double[]{-3, -3, -3};
                y = new double[]{-19, -16, -14};
                z = new double[]{90, 90, 90};

                interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, .3);
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

                UniversalConstants.kS = 0.75;

                clamp.requestState(Clamp.ClampState.COAST);

                x = new double[]{-3, -25, -41, -45};
                y = new double[]{-14, -13, -14, -16};
                z = new double[]{90, 90, 90, 90};

                interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, .3);
                swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

                //Moves across field to foundation
                while (opModeIsActive()) {
                    if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                        break;
                    }
                    jewelSwatter.requestState(JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT);
                    swerveDrive.swerveKinematics.update();
                    swerveDrive.stanleyPursuit();
                    robotDebugger.log();
                }

                UniversalConstants.kS = 0.75;
                //Releases sky stone
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.DOCK_RIGHT);
                Thread.sleep(500);

                x = new double[]{-45, -41, -25, 0};
                y = new double[]{-16, -19, -21, -23};
                z = new double[]{90, 90, 90, 90};

                interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, .3);
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

                UniversalConstants.kS = 5;

                x = new double[]{0, 7, 16};
                y = new double[]{-23, -25, -30};
                z = new double[]{90, 90, 90};

                interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, .3);
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

                UniversalConstants.kS = 0.75;

                x = new double[]{16, 7, 0};
                y = new double[]{-30, -28, -25};
                z = new double[]{90, 0, 0};

                interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, 1);
                swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

                //Backs up from second sky stone
                while (opModeIsActive()) {
                    if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                        break;
                    }
                    jewelSwatter.requestState(JewelSwatter.JewelSwatterState.RIDE_HIGH_RIGHT);
                    swerveDrive.swerveKinematics.update();
                    swerveDrive.stanleyPursuit();
                    robotDebugger.log();
                }

                // ride low
                x = new double[]{0, -25, -47};
                y = new double[]{-25, -28, -28};
                z = new double[]{0, 0, 0};

                interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, .3);
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

                x = new double[]{-47, -45, -42};
                y = new double[]{-28, -28, -28};
                z = new double[]{0, 0, 0};

                interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, .3);
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

                break;
            }

            // Case A or Unknown
            case "A": {
                double[] x = {0, 0, 0};
                double[] y = {0, -12.5, -22};
                double[] z = {90, 90, 90};

                Path interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, 0.3);
                swerveDrive.requestState(SwerveState.PATH_FOLLOWING);
                telemetry.addData("Stone", "C");
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
                clamp.requestState(Clamp.ClampState.CLAMP);
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.POSSESS_RIGHT);
                Thread.sleep(500);

                x = new double[]{0, 0, 0};
                y = new double[]{-22, -21, -18};
                z = new double[]{90, 90, 90};

                interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, 0.3);
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

                clamp.requestState(Clamp.ClampState.COAST);

                x = new double[]{0, -25, -41, -43};
                y = new double[]{-18, -16, -17, -18};
                z = new double[]{90, 90, 90, 90};

                interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, 0.3);
                swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

                //Moves across field to foundation
                while (opModeIsActive()) {
                    if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                        break;
                    }
                    jewelSwatter.requestState(JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT);
                    swerveDrive.swerveKinematics.update();
                    swerveDrive.stanleyPursuit();
                    robotDebugger.log();
                }

                //Releases sky stone
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.DOCK_RIGHT);
                Thread.sleep(500);
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.STOW_ALL);
                Thread.sleep(500);

                x = new double[]{-43, -41, -25, 0};
                y = new double[]{-18, -19, -21, -23};
                z = new double[]{90, 90, 90, 90};

                interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, 0.3);
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

                UniversalConstants.kS = 3;

                x = new double[]{0, 10, 22};
                y = new double[]{-23, -25, -34};
                z = new double[]{90, 90, 90};

                interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, 0.3);
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

//                swerveDrive.module3.swivel(-20);
//                Thread.sleep(500);
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.POSSESS_RIGHT);
                Thread.sleep(500);

                UniversalConstants.kS = 0.75;

                x = new double[]{22, 15, 0};
                y = new double[]{-34, -28, -25};
                z = new double[]{90, 0, 0};

                interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, 0.3);
                swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

                //Backs up from second sky stone
                while (opModeIsActive()) {
                    if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                        break;
                    }
                    jewelSwatter.requestState(JewelSwatter.JewelSwatterState.RIDE_HIGH_RIGHT);
                    swerveDrive.swerveKinematics.update();
                    swerveDrive.stanleyPursuit();
                    robotDebugger.log();
                }


                // ride low
                x = new double[]{0, -25, -50};
                y = new double[]{-25, -28, -28};
                z = new double[]{0, 0, 0};

                interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, 0.3);
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

                x = new double[]{-50, -47, -45};
                y = new double[]{-28, -28, -28};
                z = new double[]{0, 0, 0};

                interpolant = new Path(robotDebugger, x, y, z);
                swerveDrive.setPath(interpolant, 0.3);
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

                break;
            }
        }
    }
}
