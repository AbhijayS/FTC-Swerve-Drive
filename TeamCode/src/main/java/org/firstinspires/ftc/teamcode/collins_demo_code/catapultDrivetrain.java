package org.firstinspires.ftc.teamcode.collins_demo_code;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants;

public class catapultDrivetrain {
    private LinearOpMode linearOpMode;
    private Telemetry telemetry;
    private DcMotorEx leftOne, rightOne;
    private BNO055IMU imu;

    public void status(String s) {
        telemetry.addLine(s);
        telemetry.update();
}

    public catapultDrivetrain(LinearOpMode l, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        linearOpMode = l;
        telemetry = l.telemetry;
        HardwareMap hardwareMap = l.hardwareMap;
        status("Initialized Globals");
        leftOne = (DcMotorEx) hardwareMap.dcMotor.get(UniversalConstants.leftFront);
        rightOne = (DcMotorEx) hardwareMap.dcMotor.get(UniversalConstants.rightFront);

        status("Initialized Motors");

        leftOne.setZeroPowerBehavior(zeroPowerBehavior);
        rightOne.setZeroPowerBehavior(zeroPowerBehavior);

        status("Zero Power Behavior Set");

        leftOne.setDirection(DcMotorSimple.Direction.FORWARD);
        rightOne.setDirection(DcMotorSimple.Direction.REVERSE);

        status("Directions Set");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //We don't know how this line affects code
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Don't fully know usage of this line
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        status("IMU Parameters Set");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        status("IMU Initialized");

        leftOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void updateByGamepad() {
        leftOne.setPower(ScaledPower(linearOpMode.gamepad1.left_stick_y,.75));
        rightOne.setPower(ScaledPower(linearOpMode.gamepad1.right_stick_y,.75));
    }

    public double ScaledPower(double power, double speed) {
        if (power == 0) {
            return 0;
        }
        double maxValueOfScaled = (Math.cbrt(1 - UniversalConstants.joystickDeadzone));

        if (power > 0) {
            return speed * Math.cbrt(power - UniversalConstants.joystickDeadzone) /
                    maxValueOfScaled;
        } else {
            return speed * Math.cbrt(power + UniversalConstants.joystickDeadzone) /
                    maxValueOfScaled;
        }
    }
}