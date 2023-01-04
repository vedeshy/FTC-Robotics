package org.firstinspires.ftc.teamcode.teamcode;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Locale;

@Autonomous(name="Auto: Cone Detector Alignment")
public class ConeCollectionAlignment extends LinearOpMode {
    // Handle hardware stuff...
    int width = 800;
    int height = 448;
    // store as variable here so we can access the location
    ConeDetector detector = new ConeDetector(width);
    OpenCvCamera webcam;
    public DcMotor leftFront, leftBack, rightFront, rightBack;
    public ColorSensor color;
    public DistanceSensor dist;
    @Override
    public void runOpMode() {
        // robot logic...
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        color = hardwareMap.get(ColorSensor.class, "color");
        dist = hardwareMap.get(DistanceSensor.class, "color");
        // https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/openftc/easyopencv/examples/InternalCameraExample.java
        // Initialize the back-facing camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam Right"), cameraMonitorViewId);
        webcam.openCameraDevice();
        // Use the SkystoneDetector pipeline
        // processFrame() will be called to process the frame
        webcam.setPipeline(detector);
        // Remember to change the camera rotation
        webcam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);

        //...

        ConeDetector.ConeLocation location = detector.getLocation();

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;

        sleep(2000);
        telemetry.addData("Height of Object", detector.getHeight());
        telemetry.addData("Width of Object", detector.getWidth());
        telemetry.addData("Location of Object", detector.getLocation());
        telemetry.addData("# of Contours", detector.getNum());
        telemetry.addData("# of Rect", detector.getCount());
        telemetry.addData("Left X", detector.getLeftX());
        telemetry.addData("Right X", detector.getRightX());

        Color.RGBToHSV((int) (color.red() * SCALE_FACTOR),
                (int) (color.green() * SCALE_FACTOR),
                (int) (color.blue() * SCALE_FACTOR),
                hsvValues);

        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", dist.getDistance(DistanceUnit.CM)));
        telemetry.addData("Alpha", color.alpha());
        telemetry.addData("Red  ", color.red());
        telemetry.addData("Green", color.green());
        telemetry.addData("Blue ", color.blue());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (detector.getLocation() != ConeDetector.ConeLocation.NONE) {
                // Move to the left / right
                telemetry.addData("Left X", detector.getLeftX());
                telemetry.addData("Right X", detector.getRightX());
                telemetry.addData("Location of Object", detector.getLocation());
                telemetry.update();

                while ((detector.getRightX() - detector.getLeftX()) < 180) {
                    webcam.setPipeline(detector);
                    telemetry.addData("Left X", detector.getLeftX());
                    telemetry.addData("Right X", detector.getRightX());
                    telemetry.addData("Location of Object", detector.getLocation());
                    telemetry.update();
                    if (detector.getLocation() == ConeDetector.ConeLocation.CENTER) {
                        leftFront.setPower(-0.20);
                        rightFront.setPower(0.20);
                        leftBack.setPower(-0.20);
                        rightBack.setPower(0.20);
                    } else if (detector.getLocation() == ConeDetector.ConeLocation.LEFT) {
                        leftFront.setPower(0.25);
                        rightFront.setPower(0.30);
                        leftBack.setPower(-0.25);
                        rightBack.setPower(-0.30);
                    } else if (detector.getLocation() == ConeDetector.ConeLocation.RIGHT) {
                        leftFront.setPower(-0.30);
                        rightFront.setPower(-0.25);
                        leftBack.setPower(0.30);
                        rightBack.setPower(0.25);
                    }
                }
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
            }
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", dist.getDistance(DistanceUnit.CM)));
            telemetry.update();
            sleep(5000);
            while (dist.getDistance(DistanceUnit.INCH) > 1.3) {
                telemetry.addData("Distance (cm)",
                        String.format(Locale.US, "%.02f", dist.getDistance(DistanceUnit.CM)));
                telemetry.update();
                leftFront.setPower(-0.15);
                rightFront.setPower(0.15);
                leftBack.setPower(-0.15);
                rightBack.setPower(0.15);
            }
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", dist.getDistance(DistanceUnit.CM)));
            telemetry.update();
            sleep(5000);
            break;
        }

            // more robot logic...
    }

}
