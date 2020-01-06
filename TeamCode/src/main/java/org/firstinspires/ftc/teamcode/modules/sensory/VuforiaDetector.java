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
    private final int TOLERANCE = 70;
    private VuforiaLocalizer vuforia;

    public VuforiaDetector(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // using webcam
        WebcamName webcamName = hardwareMap.get(WebcamName.class, UniversalConstants.webcamName);
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.cameraName = webcamName;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        parameters.vuforiaLicenseKey = UniversalConstants.vuforiaLicenceKey;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565,true);
        vuforia.setFrameQueueCapacity(2);

        // using phone
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        parameters.vuforiaLicenseKey = UniversalConstants.vuforiaLicenceKey;
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565,true);
//        vuforia.setFrameQueueCapacity(2);
    }

    /* camera is pointing at the first two stones in the quarry like so:
     __ __ __ __ __ __
    |__|__|__|__|__|__|
    \     /
     \   /
      ^^^
     */

    public char getPatternBlue() {
        try {
            // capture image
            Frame frame = vuforia.getFrameQueue().take();
            Image vu_img = frame.getImage(1);
            Bitmap bitmap = Bitmap.createBitmap(vu_img.getWidth(), vu_img.getHeight(), Bitmap.Config.RGB_565);
            bitmap.copyPixelsFromBuffer(vu_img.getPixels());

            // crop the image for accuracy
            int x = bitmap.getWidth()/4;
            int y = 0;
            int width = bitmap.getWidth()/2;
            int height = bitmap.getHeight();
            bitmap = Bitmap.createBitmap(bitmap,x,y,width,height);

            // scale the image down to a 2x1 image
            bitmap = Bitmap.createScaledBitmap(bitmap, 2, 1, false);

            // compare left and right sides
            int left = rgbToGray(bitmap.getPixel(0,0));
            int right = rgbToGray(bitmap.getPixel(1,0));
            int comparison = compare(left, right);
            switch (comparison) {
                case 1: return 'C';

                case -1: return 'B';

                default: return 'A';
            }
        } catch (InterruptedException ie) {
            throw new RuntimeException(ie.getMessage());
        }
    }

    /* camera is pointing at the first two stones in the quarry like so:
     __ __ __ __ __ __
    |__|__|__|__|__|__|
                \     /
                 \   /
                  ^^^
     */

    public char getPatternRed() {
        try {
            Frame frame = vuforia.getFrameQueue().take();
            Image vu_img = frame.getImage(1);
            Bitmap bitmap = Bitmap.createBitmap(vu_img.getWidth(), vu_img.getHeight(), Bitmap.Config.RGB_565);
            bitmap.copyPixelsFromBuffer(vu_img.getPixels());

            // crop the image for accuracy
            int x = bitmap.getWidth()/4;
            int y = 0;
            int width = bitmap.getWidth()/2;
            int height = bitmap.getHeight();
            bitmap = Bitmap.createBitmap(bitmap,x,y,width,height);

            // scale the image down to a 2x1 image
            bitmap = Bitmap.createScaledBitmap(bitmap, 2, 1, false);

            int left = rgbToGray(bitmap.getPixel(0,0));
            int right = rgbToGray(bitmap.getPixel(1,0));
            int comparison = compare(left, right);
            switch (comparison) {
                case 1: return 'A';

                case -1: return 'B';

                default: return 'C';
            }
        } catch (InterruptedException ie) {
            throw new RuntimeException(ie.getMessage());
        }
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
    private int compare(int a, int b) {
        int difference = a-b;
        if (difference >= TOLERANCE)
            return 1;
        else if (difference <= -TOLERANCE)
            return -1;
        else
            return 0;
    }
}
