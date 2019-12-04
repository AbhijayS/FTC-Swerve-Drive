package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;
import org.firstinspires.ftc.teamcode.common.utilities.Util;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;

import java.util.ArrayList;

@TeleOp(name = "TestOp: New Driving")
public class NewDrivingTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad g = new Gamepad(this);
        MODULE modules[] = {MODULE.LEFT_FRONT, MODULE.RIGHT_FRONT, MODULE.RIGHT_REAR, MODULE.LEFT_REAR};

//        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) UniversalConstants.Marker.getDebuggingMarkers());
//        SwerveDrive swerveDrive = new SwerveDrive(this, null);

        // modules 0 and 2 are normal
        double normal = 0;

        // modules 1 and 3 are opposite
        double opposite = 180;

        // dead-zone is between 225 and 315 degrees

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // normalize driver input to within 0 and 360
            double STR_ANGLE = Math.toDegrees(Math.atan2(g.y, g.x));
            STR_ANGLE = normalize(STR_ANGLE);

            // reset all motor directions
            for (MODULE m: modules)
                m.resetDirection();

            // opposite angle is closer to the driver input
            if (Math.abs(angleDelta(STR_ANGLE, opposite)) < 90) {
                opposite = STR_ANGLE;
                normal = normalize(STR_ANGLE + 180);

                // normal motors flip direction
                modules[0].flipDirection();
                modules[2].flipDirection();
            }

            // normal angle is closer to the driver input
            else {
                normal = STR_ANGLE;
                opposite = normalize(normal + 180);

                // opposite motors flip direction
                modules[1].flipDirection();
                modules[3].flipDirection();
            }

            // opposite angle is in the dead-zone
            if (opposite > 225 && opposite < 315) {
                for (MODULE m : modules) {
                    m.setAngle(ANGLE.NORMAL);
                }
                // swerveDrive.swivel(normal)
                // opposite motors flip direction
                modules[1].flipDirection();
                modules[3].flipDirection();
            }

            // normal angle is in the dead-zone
            else if (normal > 225 && normal < 315) {
                for (MODULE m : modules) {
                    m.setAngle(ANGLE.OPPOSITE);
                }
                // swerveDrive.swivel(opposite);
                // normal motors flip direction
                modules[0].flipDirection();
                modules[2].flipDirection();
            }

            // none are in the dead-zone
            else {
                modules[0].setAngle(ANGLE.NORMAL);
                modules[1].setAngle(ANGLE.OPPOSITE);
                modules[2].setAngle(ANGLE.NORMAL);
                modules[3].setAngle(ANGLE.OPPOSITE);
//                swerveDrive.module0.swivel(target);
//                swerveDrive.module1.swivel(opposite);
//                swerveDrive.module2.swivel(target);
//                swerveDrive.module3.swivel(opposite);
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
            telemetry.addLine(modules[0].abbr() + " " + modules[1].abbr());
            telemetry.addLine(modules[3].abbr() + " " + modules[2].abbr());
            telemetry.addLine();
            telemetry.addLine(Math.round(modules[0].getAngle()==ANGLE.NORMAL?normal:opposite) + " " + Math.round(modules[1].getAngle()==ANGLE.NORMAL?normal:opposite));
            telemetry.addLine(Math.round(modules[3].getAngle()==ANGLE.NORMAL?normal:opposite) + " " + Math.round(modules[2].getAngle()==ANGLE.NORMAL?normal:opposite));
            telemetry.addLine();
            telemetry.addLine(modules[0].getMotorDirection() + " " + modules[1].getMotorDirection());
            telemetry.addLine(modules[3].getMotorDirection() + " " + modules[2].getMotorDirection());
            telemetry.update();
            g.update();
        }


    }

    public double normalize(double angle) {
        // 0 - 360
        return ((angle % 360) + 360) % 360;
    }

    public double scaleAngle(double angle) {
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
