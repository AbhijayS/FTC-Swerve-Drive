package org.firstinspires.ftc.teamcode.modules.swerve;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Pose;
import org.firstinspires.ftc.teamcode.common.utilities.Stopwatch;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.ModuleConfig;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.driveGearRatio;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.fudgeFactor;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.servoDefaultAngle;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.ticksPerRevolution;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.wheelCircumference;

public class SwerveKinematics {
    private Stopwatch stopwatch;
    private SwerveDrive swerveDrive;
    public SwerveModule[] swerveModules;
    private Pose centerOfMass;
    public Pose[] modulesPose; // Describes pose (position & direction) of each module in the global reference frame
    public Double[] wheelStamps; // Save the position of each module wheel for later calculations
    private double yawStamp; // Save the drivetrain's yaw
    private Double[] dS;
    private double dT;
    private double velocity;
    private double acceleration;
    private Debugger debugger;
    private BNO055IMU imu;
    private double IMU_ZERO;
    private ExpansionHubEx expansionHubBottom;
    private RevBulkData bulkDataBottom;


    public SwerveKinematics(LinearOpMode linearOpMode, Debugger debugger, SwerveDrive swerveDrive) {
        ModuleConfig[] modulesConfig = ModuleConfig.values();
        this.expansionHubBottom = linearOpMode.hardwareMap.get(ExpansionHubEx.class, UniversalConstants.expansionHubBottom);
        this.bulkDataBottom = expansionHubBottom.getBulkInputData();
        this.debugger = debugger;
        this.stopwatch = new Stopwatch();
        this.stopwatch.start();
        this.swerveDrive = swerveDrive;
        this.swerveModules = new SwerveModule[4];
        this.swerveModules[0] = swerveDrive.module0;
        this.swerveModules[1] = swerveDrive.module1;
        this.swerveModules[2] = swerveDrive.module2;
        this.swerveModules[3] = swerveDrive.module3;
        this.wheelStamps = new Double[] {
                bulkDataBottom.getMotorCurrentPosition(swerveModules[0].driveMotor) * wheelCircumference * driveGearRatio / ticksPerRevolution,
                bulkDataBottom.getMotorCurrentPosition(swerveModules[1].driveMotor) * wheelCircumference * driveGearRatio / ticksPerRevolution,
                bulkDataBottom.getMotorCurrentPosition(swerveModules[2].driveMotor) * wheelCircumference * driveGearRatio / ticksPerRevolution,
                bulkDataBottom.getMotorCurrentPosition(swerveModules[3].driveMotor) * wheelCircumference * driveGearRatio / ticksPerRevolution,
//                swerveModules[0].getDisplacement(),
//                swerveModules[1].getDisplacement(),
//                swerveModules[2].getDisplacement(),
//                swerveModules[3].getDisplacement()
        };
        this.modulesPose = new Pose[]{
                new Pose(modulesConfig[0].x, modulesConfig[0].y, servoDefaultAngle),
                new Pose(modulesConfig[1].x, modulesConfig[1].y, servoDefaultAngle),
                new Pose(modulesConfig[2].x, modulesConfig[2].y, servoDefaultAngle),
                new Pose(modulesConfig[3].x, modulesConfig[3].y, servoDefaultAngle),
//                swerveModules[0].getPose(),
//                swerveModules[1].getPose(),
//                swerveModules[2].getPose(),
//                swerveModules[3].getPose()
        };
        this.dS = new Double[]{
                0.0,
                0.0,
                0.0,
                0.0
        };
        // Define and Initialize REV IMU sensor
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = linearOpMode.hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // make sure the imu gyro is calibrated before continuing.
        while (!linearOpMode.isStopRequested() && !imu.isGyroCalibrated()) {
            linearOpMode.sleep(50);
            linearOpMode.idle();
        }
        // save IMU "zero" value in case it's non-zero after calibration
        IMU_ZERO = imu.getAngularOrientation().firstAngle;

        this.yawStamp = yaw();
        this.centerOfMass = new Pose(0, 0, 90 + this.yawStamp);
        this.velocity = 0;

        linearOpMode.telemetry.addLine("Swerve Kinematics Calibrated!");
    }


    // measure -> control -> measure -> control -> ...
    public void update() {
        bulkDataBottom = expansionHubBottom.getBulkInputData();

        stopwatch.stop();
        dT = stopwatch.seconds();
        stopwatch.start();

        double currentYaw = yaw();
        double angularDeltaDegrees = currentYaw - yawStamp;
        angularDeltaDegrees += (angularDeltaDegrees > 180) ? -360 : (angularDeltaDegrees < -180) ? 360 : 0;
        double angularDeltaRadians = Math.toRadians(angularDeltaDegrees);
        // Update Module pose 1 of 3
        // Module pose at the beginning of the last control signal
        // @see Update Module pose 3 of 3
        modulesPose[0].setHeading(swerveModules[0].getServoPosition() + yawStamp);
        modulesPose[1].setHeading(swerveModules[1].getServoPosition() + yawStamp);
        modulesPose[2].setHeading(swerveModules[2].getServoPosition() + yawStamp);
        modulesPose[3].setHeading(swerveModules[3].getServoPosition() + yawStamp);

        double[] temp = {
                bulkDataBottom.getMotorCurrentPosition(swerveModules[0].driveMotor) * wheelCircumference * driveGearRatio / ticksPerRevolution,
                bulkDataBottom.getMotorCurrentPosition(swerveModules[1].driveMotor) * wheelCircumference * driveGearRatio / ticksPerRevolution,
                bulkDataBottom.getMotorCurrentPosition(swerveModules[2].driveMotor) * wheelCircumference * driveGearRatio / ticksPerRevolution,
                bulkDataBottom.getMotorCurrentPosition(swerveModules[3].driveMotor) * wheelCircumference * driveGearRatio / ticksPerRevolution,
//                swerveModules[0].getDisplacement(),
//                swerveModules[1].getDisplacement(),
//                swerveModules[2].getDisplacement(),
//                swerveModules[3].getDisplacement()
        };
        dS = new Double[] {
                (temp[0] - wheelStamps[0]) * swerveModules[0].getMotorDirection().getSign() * fudgeFactor,
                (temp[1] - wheelStamps[1]) * swerveModules[1].getMotorDirection().getSign() * fudgeFactor,
                (temp[2] - wheelStamps[2]) * swerveModules[2].getMotorDirection().getSign() * fudgeFactor,
                (temp[3] - wheelStamps[3]) * swerveModules[3].getMotorDirection().getSign() * fudgeFactor
        };
        Double[] k = {
                angularDeltaRadians / dS[0],
                angularDeltaRadians / dS[1],
                angularDeltaRadians / dS[2],
                angularDeltaRadians / dS[3]
        };

        // update module pose
        Double[] integral0 = integrate(k[0], modulesPose[0].getRadians(), 0, dS[0]);
        Double[] integral1 = integrate(k[1], modulesPose[1].getRadians(), 0, dS[1]);
        Double[] integral2 = integrate(k[2], modulesPose[2].getRadians(), 0, dS[2]);
        Double[] integral3 = integrate(k[3], modulesPose[3].getRadians(), 0, dS[3]);

        // Update Module pose 2 of 3
        // Temporarily set the pose for each module
        // Note that these positions include errors such as wheel slip
        // and do not reflect the real robot dimensions, therefore they may be skewed.
        // Once the CoM location is averaged using these temporary values,
        // each module pose will be re-calculated to maintain the robot's physical dimensions.
        modulesPose[0].translateBy(
                integral0[0],
                integral0[1]
        );
        modulesPose[1].translateBy(
                integral1[0],
                integral1[1]
        );
        modulesPose[2].translateBy(
                integral2[0],
                integral2[1]
        );
        modulesPose[3].translateBy(
                integral3[0],
                integral3[1]
        );

        // calculate COM per wheel
        Pose[] coms = new Pose[]{new Pose(), new Pose(), new Pose(), new Pose()};
        for (int i = 0; i < modulesPose.length; i++) {
            ModuleConfig module = ModuleConfig.values()[i];
            // TODO The distance 'd' and angle 'a' could be pre-computed to save time
            double d = Math.hypot(module.x, module.y); // Distance between module and centerOfMass
            double a = Math.atan2(-module.y, -module.x); // Angle of centerOfMass from the wheel
            a += Math.toRadians(currentYaw);

            double x = d * Math.cos(a) + modulesPose[i].getX();
            double y = d * Math.sin(a) + modulesPose[i].getY();
            coms[i].setCoordinates(x, y);
        }

        // average center of mass x and y coordinates
        double comX = 0;
        double comY = 0;
        for (Pose p : coms) {
            comX += p.getX();
            comY += p.getY();
        }
        comX /= 4;
        comY /= 4;

        // calculate velocity
        double dX = Math.hypot(comX - centerOfMass.getX(), comY - centerOfMass.getY());
        acceleration = ((dX/dT) - velocity)/dT;
        velocity = dX / dT;

//        double v1 = bulkDataBottom.getMotorVelocity(swerveModules[0].driveMotor);
//        double v2 = bulkDataBottom.getMotorVelocity(swerveModules[1].driveMotor);
//        double v3 = bulkDataBottom.getMotorVelocity(swerveModules[2].driveMotor);
//        double vAvg = convertTicksToInches((v1+v2+v3)/3);
//        acceleration = (vAvg-velocity)/dT;
//        velocity = vAvg;

        // update center of mass coordinates
        centerOfMass.setCoordinates(comX, comY);
        centerOfMass.setPose(comX, comY, centerOfMass.getDegrees() + angularDeltaDegrees);

        // re-calculate module pose using the averaged CoM coordinates
        for (int i = 0; i < modulesPose.length; i++) {
            ModuleConfig module = ModuleConfig.values()[i];

            // TODO The distance 'd' and angle 'a' could be pre-computed to save time
            double d = Math.hypot(module.x, module.y);
            double a = Math.atan2(module.y, module.x);
            a += Math.toRadians(currentYaw);

            double x = d * Math.cos(a) + centerOfMass.getX();
            double y = d * Math.sin(a) + centerOfMass.getY();

            // Update Module pose 3 of 3
            // Update module pose by considering the CoM
            // and angular delta since the last control signal.
            modulesPose[i].setPose(
                    x,
                    y,
                    modulesPose[i].getDegrees() + angularDeltaRadians
            );
        }

        yawStamp = currentYaw;
        this.wheelStamps = new Double[]{
                temp[0],
                temp[1],
                temp[2],
                temp[3]
        };

        try {
            debugger.addData(Debugger.Marker.RX.toString(), Double.toString(centerOfMass.getX()));
            debugger.addData(Debugger.Marker.RY.toString(), Double.toString(centerOfMass.getY()));
            debugger.addData(Debugger.Marker.HEADING.toString(), Double.toString(centerOfMass.getDegrees()));
        } catch (Exception e) {}
    }

    public double getVelocity() {
        if (Double.isNaN(velocity))
            return 0;
        return velocity;
    }

    public double getAcceleration() {
        if (Double.isNaN(acceleration))
            return 0;
        return acceleration;
    }

    public Pose getCenterOfMass() {
        return new Pose(centerOfMass.getX(), centerOfMass.getY(), centerOfMass.getDegrees());
    }

    // (1/k) * sin(k*S + C) | s1->s2
    // (-1/k) * cos(k*S + C) | s1->s2
    private Double[] integrate(Double k, double C, double s1, double s2) {
        if (k == 0) {
            return new Double[]{s2 * Math.cos(C), s2 * Math.sin(C)};
        }

        if (k.isInfinite() || k.isNaN()) {
            return new Double[]{0.0, 0.0};
        }

        double upperX = (1 / k) * Math.sin(k * s2 + C);
        double lowerX = (1 / k) * Math.sin(k * s1 + C);
        double upperY = (-1 / k) * Math.cos(k * s2 + C);
        double lowerY = (-1 / k) * Math.cos(k * s1 + C);
        return new Double[]{upperX - lowerX, upperY - lowerY};
    }

    private double yaw() {
        double angle = imu.getAngularOrientation().firstAngle;
        double delta = angle - IMU_ZERO;

        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;

        return delta;
    }

    private double convertTicksToInches(double ticks) {
        return ticks * wheelCircumference * driveGearRatio / ticksPerRevolution;
    }

    public void resetGyro() {
        IMU_ZERO = imu.getAngularOrientation().firstAngle;
        centerOfMass.setHeading(90);
    }

    public double getYaw() {
        return yawStamp;
    }

    public double getRawIMU() {
        return imu.getAngularOrientation().firstAngle;
    }

    public double getIMU_ZERO() {
        return IMU_ZERO;
    }

}
