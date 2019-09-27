package org.firstinspires.ftc.teamcode.RobotModules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

public class TensorFlowLite {

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private Telemetry telemetry;
    private LinearOpMode linearOpMode;
    private HardwareMap hardwareMap;
    String pattern;


    public void status(String s) {
        telemetry.addLine(s);
        telemetry.update();
    }

    public TensorFlowLite(LinearOpMode l) {
        linearOpMode = l;
        telemetry = linearOpMode.telemetry;
        hardwareMap = linearOpMode.hardwareMap;
        pattern = "Unknown";


        initVuforia();

        status("Vuforia has been initialized.");

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
            status("Tensor Flow has been initialized.");
        } else {
            status("This device does not support Tensor Flow.");
        }
    }


    public void activateTfod() {
        if (tfod != null) {
            tfod.activate();
        }
    }

    public void shutDownTfod() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public void determinePattern() {
        if (tfod != null) {
            List<Recognition> updateRecognitions = tfod.getUpdatedRecognitions();
            if (updateRecognitions != null) {
                telemetry.addData("Objects Detected", updateRecognitions.size());
                if (updateRecognitions.size() >= 3) {
                    int SkyStoneX = -1;
                    int SkyStoneY = -1;
                    int Stone1X = -1;
                    int Stone1Y = -1;
                    int Stone2X = -1;
                    int Stone2Y = -1;

                    ArrayList<Integer> SkystoneXvals = new ArrayList<Integer>();
                    ArrayList<Integer> SkystoneYvals = new ArrayList<Integer>();
                    ArrayList<Integer> StoneXvals = new ArrayList<Integer>();
                    ArrayList<Integer> StoneYvals = new ArrayList<Integer>();
                    for (Recognition recognition : updateRecognitions) {
                        if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                            SkystoneXvals.add((int) recognition.getLeft());
                            SkystoneYvals.add((int) recognition.getBottom());
                        } else {
                            StoneXvals.add((int) recognition.getLeft());
                            StoneYvals.add((int) recognition.getBottom());
                        }
                    }
                    int indexToRemove = 0;
                    for (int i = 0; i < SkystoneYvals.size(); i++) {
                        if (SkyStoneY <= SkystoneYvals.get(i) && SkyStoneX < SkystoneXvals.get(i)) {
                            goldMineralY = goldYvals.get(i);
                            goldMineralX = goldXvals.get(i);
                        }
                    }

                    for (int i = 0; i < silverYvals.size(); i++) {
                        if (silverMineral1Y < silverYvals.get(i)) {
                            silverMineral1Y = silverYvals.get(i);
                            silverMineral1X = silverXvals.get(i);
                            indexToRemove = i;
                        }
                    }
                    if (silverYvals.size() > 0) {
                        silverXvals.remove(indexToRemove);
                        silverYvals.remove(indexToRemove);
                    }
                    for (int i = 0; i < silverYvals.size(); i++) {
                        if (silverMineral2Y < silverYvals.get(i)) {
                            silverMineral2Y = silverYvals.get(i);
                            silverMineral2X = silverXvals.get(i);
                        }
                    }
                }
            }
        }
    }


    private void initVuforia() {
        //This method will create all neccessary vuforia parameters and set them
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = UniversalConstants.vuforiaLicenceKey;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initVuforiaWebCam() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = UniversalConstants.vuforiaLicenceKey;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
