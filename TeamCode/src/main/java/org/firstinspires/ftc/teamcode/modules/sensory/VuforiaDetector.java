package org.firstinspires.ftc.teamcode.modules.sensory;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.common.UniversalConstants;

public class VuforiaDetector {
    private VuforiaLocalizer vuforia;

    public VuforiaDetector(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.cameraName = webcamName;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey = UniversalConstants.vuforiaLicenceKey;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565,true);
        vuforia.setFrameQueueCapacity(2);
    }

    public char getPatternBlue() {
        try {
            Frame frame = vuforia.getFrameQueue().take();
            Image vu_img = frame.getImage(1);
            Bitmap bitmap = Bitmap.createBitmap(vu_img.getWidth(), vu_img.getHeight(), Bitmap.Config.RGB_565);
            bitmap.copyPixelsFromBuffer(vu_img.getPixels());
            bitmap = Bitmap.createScaledBitmap(bitmap, 2, 1, false);
            int height = bitmap.getHeight();
            int width = bitmap.getWidth();
            int left = rgbToGray(bitmap.getPixel(0,0));
            int right = rgbToGray(bitmap.getPixel(1,0));
            int comparison = compare(left, right);
        } catch (InterruptedException ie) {
            throw new RuntimeException(ie.getMessage());
        }
        return 'C';
    }

    private int rgbToGray(int color) {
        int R = (color >> 16) & 0xff;
        int G = (color >>  8) & 0xff;
        int B = (color      ) & 0xff;
        int gray = (int) Math.round(0.3*R + 0.59*G + 0.11*B);
        return gray;
    }

    /*
    Roughly:
    a > b = 1
    a < b = -1
    a == b = 0
     */
    // TODO: Improve this function with some tolerance
    private int compare(int a, int b) {
        return Integer.compare(a, b);
    }
}
