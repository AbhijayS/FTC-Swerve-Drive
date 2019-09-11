package org.firstinspires.ftc.teamcode.RobotModules;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SwereveDrive {
    private LinearOpMode linearOpMode;
    private Telemetry telemetry;
    private DcMotorEx leftFront, rightFront, rightRear, leftRear;
    private PIDCoefficients pidCoefficients;
    private BNO055IMU imu;

    private DriveSpeed driveSpeed;

    enum DriveSpeed {
        FULLSPEED, HALFSPEED, QUARTERSPEED
    }

    public void status(String s) {
        telemetry.addLine(s);
        telemetry.update();
    }

    public SwereveDrive(LinearOpMode l, DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        linearOpMode = l;
        telemetry = linearOpMode.telemetry;
        driveSpeed = DriveSpeed.FULLSPEED;
        HardwareMap hardwareMap = linearOpMode.hardwareMap;

        status("Globals Initialized");

        leftFront = (DcMotorEx) hardwareMap.dcMotor.get(UniversalConstants.leftFront);
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get(UniversalConstants.rightFront);
        leftRear = (DcMotorEx) hardwareMap.dcMotor.get(UniversalConstants.leftRear);
        rightRear = (DcMotorEx) hardwareMap.dcMotor.get(UniversalConstants.rightRear);

        status("Initialized Motors");

        leftFront.setZeroPowerBehavior(zeroPowerBehavior);
        rightFront.setZeroPowerBehavior(zeroPowerBehavior);
        leftRear.setZeroPowerBehavior(zeroPowerBehavior);
        rightRear.setZeroPowerBehavior(zeroPowerBehavior);

        status("Zero Power Behavior Set");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        pidCoefficients = new PIDCoefficients(3, .1140625 * 2, 1.875);
        leftFront.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidCoefficients);
        leftRear.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidCoefficients);
        rightFront.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidCoefficients);
        rightRear.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidCoefficients);

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

        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void setRunMode(DcMotor.RunMode runMode){
        leftRear.setMode(runMode);
        leftFront.setMode(runMode);
        rightRear.setMode(runMode);
        rightFront.setMode(runMode);
    }



}
