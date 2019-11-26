package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveKinematics;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.clipAngle;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.kI;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.kP;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.roundTo2DecimalPlaces;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.tolerance;

@TeleOp(name = "TestOp: Turn PID")
public class TurnPID extends LinearOpMode {
    SwerveKinematics swerveKinematics;
    double ce = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        SwerveDrive swerveDrive = new SwerveDrive(this,null);
        swerveKinematics = swerveDrive.swerveKinematics;
        double target = 0;

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double degrees = swerveKinematics.getCenterOfMass().getDegrees();
            double power;
            if (Math.abs(degrees-target) > 1) {
                power = turnPID(target);
            } else
                power = 0;
            swerveDrive.fod(90,0,power,swerveKinematics.getYaw());
            telemetry.addData("Degrees", degrees);
            telemetry.addData("Power", power);
            telemetry.addData("Cum. Error", ce);
            swerveKinematics.update();
            telemetry.update();
        }

    }
    public double turnPID(double targetAngle) {
        double heading = roundTo2DecimalPlaces(swerveKinematics.getCenterOfMass().getDegrees());
        double setAngle = roundTo2DecimalPlaces(clipAngle(targetAngle));
        double err = roundTo2DecimalPlaces(heading - setAngle);
        err += (err > 180) ? -360 : (err < -180) ? 360 : 0;
        ce += err;
        double power = Math.abs(err) <= tolerance ? 0 : err * kP + ce * kI;
        return power;
    }
}
