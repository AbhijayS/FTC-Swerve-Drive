package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;
import org.firstinspires.ftc.teamcode.common.utilities.Util;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveModule;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.ModuleConfig.MODULE_ONE;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.ModuleConfig.MODULE_THREE;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.ModuleConfig.MODULE_TWO;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.ModuleConfig.MODULE_ZERO;

@TeleOp(name = "TestOp: Opposite Driving")
public class OppositeDriving extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad g = new Gamepad(this);
        MODULE modules[] = {MODULE.LEFT_FRONT, MODULE.RIGHT_FRONT, MODULE.RIGHT_REAR, MODULE.LEFT_REAR};
        SwerveDrive swerveDrive = new SwerveDrive(this, null);
        SwerveModule s1 = new SwerveModule(hardwareMap,MODULE_ONE,swerveDrive);

        double delta = 3.8;

        // modules 0 and 2 are normal
        double normal = 0;

        // modules 1 and 3 are opposite
        double opposite = 180;

        // dead-zone is between 225 and 315 degrees

        telemetry.update();
        waitForStart();

//        double STR_ANGLE = 180;
        while (opModeIsActive()) {

            // normalize driver input to within 0 and 360
            double STR_ANGLE = Math.toDegrees(Math.atan2(g.y, g.x));
            STR_ANGLE = normalize(STR_ANGLE);

            // reset all motor directions
            for (MODULE m: modules) {
                m.resetDirection();
            }
//            for (SwerveModule s : swerveModules) {
//                s.resetMotorDirection();
//            }
            s1.resetMotorDirection();

            // opposite angle is closer to the driver input
            if (Math.abs(angleDelta(STR_ANGLE, opposite)) < 90) {
                opposite = STR_ANGLE;
                normal = normalize(STR_ANGLE + 180);
                // normal motors flip direction
//                modules[0].flipDirection();
//                modules[2].flipDirection();
//                swerveModules[0].reverseMotorDirection();
//                swerveModules[2].reverseMotorDirection();
//                s1.reverseMotorDirection();
                telemetry.addLine("Opposite is closer");
            }

            // normal angle is closer to the driver input
            else {
                normal = STR_ANGLE;
                opposite = normalize(normal + 180);
                // opposite motors flip direction
                modules[1].flipDirection();
                modules[3].flipDirection();
                s1.reverseMotorDirection();
//                swerveModules[1].reverseMotorDirection();
//                swerveModules[3].reverseMotorDirection();
                telemetry.addLine("Normal is closer");
            }

            normal = normalize(normal + delta);
            opposite = normalize(opposite + delta);

//             opposite angle is in the dead-zone
            if (opposite > 225 && opposite < 315) {
                // swivel all to normal angle
                for (MODULE m : modules) {
                    m.setAngle(ANGLE.NORMAL);
                }
//                for (SwerveModule s : swerveModules) {
//                    s.setPosition(scaleAngle(normal));
//                }
                s1.setPosition(scaleAngle(normal));
                // opposite motors flip direction
                modules[1].flipDirection();
                modules[3].flipDirection();
                s1.reverseMotorDirection();
//                swerveModules[1].reverseMotorDirection();
//                swerveModules[3].reverseMotorDirection();
                telemetry.addLine("Opposite in dead-zone");
            }

//            // normal angle is in the dead-zone
//            if (normal > 225 && normal < 315) {
//                // swivel all to opposite angle
//                for (MODULE m : modules) {
//                    m.setAngle(ANGLE.OPPOSITE);
//                }
////                for (SwerveModule s : swerveModules) {
////                    s.setPosition(scaleAngle(opposite));
////                }
//                s1.setPosition(scaleAngle(opposite));
//                // normal motors flip direction
//                modules[0].flipDirection();
//                modules[2].flipDirection();
////                swerveModules[0].reverseMotorDirection();
////                swerveModules[2].reverseMotorDirection();
//                s1.reverseMotorDirection();
//                telemetry.addLine("Normal in dead-zone");
//            }

            // none are in the dead-zone
            else {
                // swivel to the respective angles
//                modules[0].setAngle(ANGLE.NORMAL);
                modules[1].setAngle(ANGLE.OPPOSITE);
//                modules[2].setAngle(ANGLE.NORMAL);
                modules[3].setAngle(ANGLE.OPPOSITE);
//                swerveModules[0].setPosition(scaleAngle(normal));
//                swerveModules[1].setPosition(scaleAngle(opposite));
//                swerveModules[2].setPosition(scaleAngle(normal));
//                swerveModules[3].setPosition(scaleAngle(opposite));
                s1.setPosition(scaleAngle(opposite));
                telemetry.addLine("Live-zone");
            }
/*

             _______   ______   _        ______   __  __   ______   _______   _____   __     __
            |__   __| |  ____| | |      |  ____| |  \/  | |  ____| |__   __| |  __ \  \ \   / /
               | |    | |__    | |      | |__    | \  / | | |__       | |    | |__) |  \ \_/ /
               | |    |  __|   | |      |  __|   | |\/| | |  __|      | |    |  _  /    \   /
               | |    | |____  | |____  | |____  | |  | | | |____     | |    | | \ \     | |
               |_|    |______| |______| |______| |_|  |_| |______|    |_|    |_|  \_\    |_|


 */
            telemetry.addData("INPUT", (STR_ANGLE));
//            telemetry.addData("Servo", scaleAngle(STR_ANGLE));
//            telemetry.addLine(modules[0].abbr() + " " + modules[1].abbr());
//            telemetry.addLine(modules[3].abbr() + " " + modules[2].abbr());
//            telemetry.addLine();
//            telemetry.addLine(Math.round(modules[0].getAngle()==ANGLE.NORMAL?normal:opposite) + " " + Math.round(modules[1].getAngle()==ANGLE.NORMAL?normal:opposite));
//            telemetry.addLine(Math.round(modules[3].getAngle()==ANGLE.NORMAL?normal:opposite) + " " + Math.round(modules[2].getAngle()==ANGLE.NORMAL?normal:opposite));
//            telemetry.addLine();
//            telemetry.addLine(modules[0].getMotorDirection() + " " + modules[1].getMotorDirection());
//            telemetry.addLine(modules[3].getMotorDirection() + " " + modules[2].getMotorDirection());
            telemetry.addData("Sim", modules[1].getMotorDirection());
            telemetry.addData("Real", s1.getMotorDirection().toString());
            telemetry.update();
            g.update();
            s1.setPower(0.1);
        }
    }

    public double normalize(double angle) {
        // 0 - 360
        return ((angle % 360) + 360) % 360;
    }

    public double scaleAngle(double angle) {
        // 225 ==> 0
        // 315 ==> 1
        if (angle >= 315)
            angle -= 360;
        return (225 - angle) / 270;
    }

    public double angleDelta(double target, double source) {
        double diff = target - source;
        diff += (diff > 180) ? -360 : (diff < -180) ? 360 : 0;
        return diff;
    }

    enum ANGLE {
        NORMAL, OPPOSITE
    }

    enum DIRECTION {
        FORWARD, REVERSE
    }

    // solely for debugging purposes
    enum MODULE {
        LEFT_FRONT(ANGLE.NORMAL, DIRECTION.FORWARD),
        RIGHT_FRONT(ANGLE.OPPOSITE, DIRECTION.FORWARD),
        RIGHT_REAR(ANGLE.NORMAL, DIRECTION.FORWARD),
        LEFT_REAR(ANGLE.OPPOSITE, DIRECTION.FORWARD);

        ANGLE angle;
        DIRECTION motorDirection;

        MODULE(ANGLE angle, DIRECTION direction) {
            setAngle(angle);
            setMotorDirection(direction);

        }

        public ANGLE getAngle() {
            return angle;
        }

        public void setAngle(ANGLE angle) {
            this.angle = angle;
        }

        public void setMotorDirection(DIRECTION motorDirection) {
            this.motorDirection = motorDirection;
        }

        public void flipDirection() {
            if (this.motorDirection == DIRECTION.FORWARD)
                this.motorDirection = DIRECTION.REVERSE;
            else
                this.motorDirection = DIRECTION.FORWARD;
        }

        public void resetDirection() {
            this.motorDirection = DIRECTION.FORWARD;
        }

        @Override
        public String toString() {
            return this.name() + " " + getAngle();
        }

        public char abbr() {
            return this.getAngle().toString().charAt(0);
        }

        public char getMotorDirection() {
            return motorDirection.toString().charAt(0);
        }
    }
}
