package org.firstinspires.ftc.teamcode.teamcode;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Auto: Cone Detector")
public class ConeCollection extends LinearOpMode {
    // Handle hardware stuff...
    int width = 800;
    int height = 448;
    // store as variable here so we can access the location
    ConeDetector detector = new ConeDetector(width);
    OpenCvCamera webcam;
    public DcMotor leftFront, leftBack, rightFront, rightBack;
    @Override
    public void runOpMode() {
        // robot logic...
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
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

        sleep(2000);
        telemetry.addData("Height of Object", detector.getHeight());
        telemetry.addData("Width of Object", detector.getWidth());
        telemetry.addData("Location of Object", detector.getLocation());
        telemetry.addData("# of Contours", detector.getNum());
        telemetry.addData("# of Rect", detector.getCount());
        telemetry.addData("Left X", detector.getLeftX());
        telemetry.addData("Right X", detector.getRightX());
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
                        leftFront.setPower(-0.20);
                        rightFront.setPower(0.40);
                        leftBack.setPower(-0.20);
                        rightBack.setPower(0.40);
                    } else if (detector.getLocation() == ConeDetector.ConeLocation.RIGHT) {
                        leftFront.setPower(-0.27);
                        rightFront.setPower(0.20);
                        leftBack.setPower(-0.27);
                        rightBack.setPower(0.20);
                    }
                }
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
            }
        }

            // more robot logic...
    }

}
