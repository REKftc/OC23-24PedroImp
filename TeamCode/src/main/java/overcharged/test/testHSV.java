package overcharged.test;

import static overcharged.config.RobotConstants.TAG_A;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import overcharged.components.propLocation;
import overcharged.linear.util.WaitLinear;

@Autonomous(name="hsv")
public class testHSV extends LinearOpMode {
    HSVPipeline detector;
    OpenCvWebcam webcam;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry telems;
    private propLocation location = propLocation.Middle;

    @Override

    public void runOpMode() throws InterruptedException {
        telems = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);
        WaitLinear lp = new WaitLinear(this);
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        detector = new HSVPipeline();
        webcam.setPipeline(detector);
     //   webcam.openCameraDevice();
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened() //calls when camera opens
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
            }
            @Override
            public void onError(int errorCode) //calls if camera did not open
            {
                System.out.println(errorCode);
            }
        });
        waitForStart();

        if (opModeIsActive()) {
            lp.waitMillis(2000);
            location = detector.getLocation(true, false);
            //RobotLog.ii(TAG_A, "color red?", detector.getColor());
            RobotLog.ii(TAG_A, "avg color?", detector.getAverageColor());
            RobotLog.ii(TAG_A, "location? " + location);
            //telemetry.addLine("color red? " + detector.getColor());
            telemetry.addLine("mid avg color? " + detector.getAverageColor());
            telemetry.addLine("r avg color? " + detector.getRightAverageColor());
            telemetry.addLine("b avg color? " + detector.getBlueAverageColor());
            telemetry.addLine("bl avg color? " + detector.getBlueLeftAverageColor());
            telemetry.addLine("red far location? " + location);
            telemetry.addLine("red close location? " + detector.getLocation(true, true));
            telemetry.addLine("blue far location? " + detector.getLocation(false, false));
            telemetry.addLine("blue close location? " + detector.getLocation(false, true));

            telemetry.update();
            lp.waitMillis(5000);
        }
        }

    }


