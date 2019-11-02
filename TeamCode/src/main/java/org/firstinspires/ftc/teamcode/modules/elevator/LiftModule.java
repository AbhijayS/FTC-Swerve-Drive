package org.firstinspires.ftc.teamcode.modules.elevator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.states.PositionalStates;
import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;

public class LiftModule {
    //REV Orbital 20
    private LinearOpMode linearOpMode;
    private Telemetry telemetry;
    private DcMotorEx liftOne, liftTwo;
    public PositionalStates state;
    private PIDCoefficients pidCoefficients;
    private boolean runPosition = false;
    private boolean holdPosition = true;
    private double incremHeight = 5;
    private int increment = 0;


    public void status(String s) {
        telemetry.addLine(s);
        telemetry.update();
    }


    /**
     * Constructor for Lift Module
     *
     * @param l Passes in Linear op mode from Tele-Op class
     */
    public LiftModule(LinearOpMode l) {
        linearOpMode = l;
        telemetry = linearOpMode.telemetry;
        state = PositionalStates.FULL;
        HardwareMap hardwareMap = l.hardwareMap;

        status("Globals Initialized");

        liftOne = (DcMotorEx) hardwareMap.dcMotor.get("L1");
        liftTwo = (DcMotorEx) hardwareMap.dcMotor.get("L2");

        status("Motors Initialized");

        liftOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        status("Zero Power Behavior Set");

        liftOne.setDirection(DcMotorSimple.Direction.FORWARD);
        liftTwo.setDirection(DcMotorSimple.Direction.FORWARD);
        //pidCoefficients = new PIDCoefficients(0, 0, 0);
        //liftOne.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidCoefficients);
        //liftTwo.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidCoefficients);

        status("Directions Set");
        liftTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void setRunMode(DcMotor.RunMode runMode) {
        liftOne.setMode(runMode);
        liftTwo.setMode(runMode);
    }

    public double convertToTicks(double inches) {
        double tickValue = 0;
        double linear_conversion = inches / 4.0;
        tickValue = linear_conversion * (1.0 / (2.3 * Math.PI)) * 62 * (38.0 / 62.0) * (1.0 / 38.0) * 560.0;
        return tickValue;
    }

    public double convertToInches(double Tics) {
        double height = 0;
        double linear_conversion = Tics / ((1.0 / (2.3 * Math.PI)) * 62 * (38.0 / 62.0) * (1.0 / 38.0) * 560.0);
        height = 4 * linear_conversion;
        return height;
    }

    public void moveHeight(double height, double power) {
        if (height <= 42) {
            double p = power;
            //setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(((int)convertToTicks(height))<liftTwo.getCurrentPosition()){
                p = -p;
            }
            liftOne.setPower(p);
            liftTwo.setPower(p);
            liftOne.setTargetPosition((int) convertToTicks(height));
            liftTwo.setTargetPosition((int) convertToTicks(height));
            telemetry.addData("Height: ", height);
            telemetry.addData("Height Conversion: ", (int) convertToTicks(height));
            telemetry.addData("LiftOne Pos: ", liftOne.getCurrentPosition());
            telemetry.addData("Target Position: ", liftTwo.getTargetPosition());
            telemetry.addData("LiftTwo Pos: ", liftTwo.getCurrentPosition());
        }
        //double averagePosition = (liftOne.getCurrentPosition() + liftTwo.getTargetPosition()) / 2.0;
        if (liftTwo.getCurrentPosition() <= (liftTwo.getTargetPosition() + 20) && liftTwo.getCurrentPosition() >= (liftTwo.getTargetPosition() - 20)) {
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            runPosition = false;

        }

    }

    public void moveToState() {
        if (PositionalStates.FULL.height <= 54) {
            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftOne.setPower(state.runPower);
            liftTwo.setPower(state.runPower);
            //liftOne.setTargetPosition((int) convertToTicks(state.height));
            liftTwo.setTargetPosition((int) convertToTicks(state.height));
            telemetry.addData("Height: ", state.height);
            telemetry.addData("Height Conversion: ", (int) convertToTicks(state.height));
            //telemetry.addData("LiftOne Pos: ", liftOne.getCurrentPosition());
            telemetry.addData("LiftTwo Pos: ", liftTwo.getCurrentPosition());
        }
        //double averagePosition = (liftOne.getCurrentPosition() + liftTwo.getTargetPosition()) / 2.0;
        if (liftTwo.getCurrentPosition() <= liftTwo.getTargetPosition() + 20 && liftTwo.getCurrentPosition() >= liftTwo.getTargetPosition() - 20) {
            runPosition = false;
            //return true;
        }
        //return false;
    }

    public void updateByGamepad(Gamepad g, double stick) {

        // This sets the joystick to control the power with a cubic root function and caps the value at the max power of 1
        double power = Range.clip(Math.cbrt(stick), -.5, .5);
        /*if (liftTwo.getCurrentPosition() <= convertToTicks(42)+10 && liftTwo.getCurrentPosition() >= convertToTicks(0) - 10) {
            liftOne.setPower(power);
            liftTwo.setPower(power);
        }*/
        /*if (stick == 0) {
            holdPosition = true;
        } else if (stick != 0) {
            holdPosition = false;
        }*/


        if (liftTwo.getCurrentPosition() >= convertToTicks(54) && power > 0) {
            power = 0;
        } else if (liftTwo.getCurrentPosition() <= convertToTicks(0) && power < 0) {
            power = 0;
        }
        /*if (power == 0) {
            liftOne.setPower(power);
            liftTwo.setPower(power);
        }*/
        if(!runPosition) {
            liftOne.setPower(power);
            liftTwo.setPower(power);
        }

        if (g.Ou) {
            //telemetry.addLine("FULL HEIGHT");
            //state = PositionalStates.FULL;
            Range.clip(increment+=1,0,10);
            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            runPosition = true;
        } else if (g.Od) {
            //telemetry.addLine("QUARTER HEIGHT");
            //state = PositionalStates.QUARTER;
            Range.clip(increment-=1,0,10);
            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            runPosition = true;
        } else if (g.Ol) {
            //telemetry.addLine("HALF HEIGHT");
            //state = PositionalStates.HALF;
            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            runPosition = true;
        } else if (g.Or) {
            //telemetry.addLine("THREE_QUARTER HEIGHT");
            //state = PositionalStates.THREE_QUARTERS;
            increment = 0;
            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            runPosition = true;
        }


        if (runPosition) {
            moveHeight(Range.clip((incremHeight * increment),0,54),.8);
            telemetry.addLine("moving to position");
        }
        /*if (!moveHeight(convertToInches(liftTwo.getCurrentPosition()), .25) && holdPosition) ;
        {
            telemetry.addLine("holding");
        }*/
        telemetry.addData("increment: ",increment);
        telemetry.addData("power: ", power);
        telemetry.addData("position: ", liftTwo.getCurrentPosition());
        //telemetry.addData("max position: ", convertToTicks(42));
        //telemetry.addData("stick value: ", stick);


    }


}
