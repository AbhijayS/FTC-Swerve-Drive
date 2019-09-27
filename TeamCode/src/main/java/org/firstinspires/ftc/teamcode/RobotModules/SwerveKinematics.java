package org.firstinspires.ftc.teamcode.RobotModules;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants.ModuleConfig;
import static org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants.servoDefaultAngle;

public class SwerveKinematics {
    private Stopwatch stopwatch;
    private SwerveDrive swerveDrive;
    private SwerveModule[] swerveModules;
    private Point centerOfMass;
    private Point[] modulesPose; // Describes pose (position & direction) of each module in the global reference frame
    private Double[] wheelStamps; // Save the position of each module wheel for later calculations
    private double yawStamp; // Save the drivetrain's yaw
    private Double[] dS;
    private double dT;
    private double velocity;
    private Debugger debugger;
    private BNO055IMU imu;
    private double IMU_ZERO;

    public SwerveKinematics(LinearOpMode linearOpMode,  SwerveDrive swerveDrive) {
        UniversalConstants.ModuleConfig[] modulesConfig = UniversalConstants.ModuleConfig.values();
        //this.debugger = debugger;
        this.stopwatch = new Stopwatch();
        this.stopwatch.start();
        this.swerveDrive = swerveDrive;
        this.swerveModules = new SwerveModule[4];
        this.swerveModules[0] = swerveDrive.module0;
        this.swerveModules[1] = swerveDrive.module1;
        this.swerveModules[2] = swerveDrive.module2;
        this.swerveModules[3] = swerveDrive.module3;
        this.wheelStamps = new Double[]{
                swerveModules[0].getDisplacement(),
                swerveModules[1].getDisplacement(),
                swerveModules[2].getDisplacement(),
                swerveModules[3].getDisplacement()
        };
        this.modulesPose = new Point[] {
                new Point(modulesConfig[0].x, modulesConfig[0].y, servoDefaultAngle),
                new Point(modulesConfig[1].x, modulesConfig[1].y, servoDefaultAngle),
                new Point(modulesConfig[2].x, modulesConfig[2].y, servoDefaultAngle),
                new Point(modulesConfig[3].x, modulesConfig[3].y, servoDefaultAngle),
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
        this.yawStamp = getYaw();
        this.centerOfMass = new Point(0, 0, 90 + this.yawStamp);
        this.velocity = 0;

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
        linearOpMode.telemetry.addLine("Swerve Kinematics Calibrated!");
    }


    // measure -> control -> measure -> control -> ...
    public void update() {

        stopwatch.stop();
        dT = stopwatch.seconds();
        stopwatch.start();

        double currentYaw = yaw();
        double angularDelta = currentYaw - yawStamp;
        angularDelta += (angularDelta > 180) ? -360 : (angularDelta < -180) ? 360 : 0;
        angularDelta = Math.toRadians(angularDelta);

        // Update Module pose 1 of 3
        // Module pose at the beginning of the last control signal
        // @see Update Module pose 3 of 3
        modulesPose[0].setHeading(swerveModules[0].getServoPosition() + yawStamp);
        modulesPose[1].setHeading(swerveModules[1].getServoPosition() + yawStamp);
        modulesPose[2].setHeading(swerveModules[2].getServoPosition() + yawStamp);
        modulesPose[3].setHeading(swerveModules[3].getServoPosition() + yawStamp);

        double[] temp = {
                swerveModules[0].getDisplacement(),
                swerveModules[1].getDisplacement(),
                swerveModules[2].getDisplacement(),
                swerveModules[3].getDisplacement()
        };
        dS = new Double[]{
                (temp[0] - wheelStamps[0]) * swerveModules[0].getMotorDirection().getSign(),
                (temp[1] - wheelStamps[1]) * swerveModules[1].getMotorDirection().getSign(),
                (temp[2] - wheelStamps[2]) * swerveModules[2].getMotorDirection().getSign(),
                (temp[3] - wheelStamps[3]) * swerveModules[3].getMotorDirection().getSign()
        };
        Double[] k = {
                angularDelta / dS[0],
                angularDelta / dS[1],
                angularDelta / dS[2],
                angularDelta / dS[3]
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
        Point[] coms = new Point[]{new Point(), new Point(), new Point(), new Point()};
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
        for (Point p : coms) {
            comX += p.getX();
            comY += p.getY();
        }
        comX /= 4;
        comY /= 4;

        // calculate velocity
        double dX = Math.hypot(comX - centerOfMass.getX(), comY - centerOfMass.getY());
        velocity = dX / dT;

        // update center of mass coordinates
        centerOfMass.setCoordinates(comX, comY);

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
                    modulesPose[i].getDegrees() + angularDelta
            );
        }

        yawStamp = currentYaw;
        this.wheelStamps = new Double[]{
                temp[0],
                temp[1],
                temp[2],
                temp[3]
        };
    }

    public double getVelocity() {
        if (Double.isNaN(velocity))
            return 0;
        return velocity;
    }

    public Point getCenterOfMass() {
        return new Point(centerOfMass.getX(), centerOfMass.getY(), centerOfMass.getDegrees());
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
