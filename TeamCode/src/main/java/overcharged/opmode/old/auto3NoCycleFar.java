package overcharged.opmode.old;


import static overcharged.config.RobotConstants.TAG_SL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import overcharged.components.RobotMecanum;
import overcharged.components.propLocation;
import overcharged.config.RobotConstants;
import overcharged.linear.util.SelectLinear;
import overcharged.linear.util.WaitLinear;
import overcharged.drive.SampleMecanumDrive;
import overcharged.opmode.vSlidesThread;
import overcharged.test.EasyOpenCVExample;
import overcharged.test.HSVPipeline;
import overcharged.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(name="auto3NoCycleFar")
public class auto3NoCycleFar extends LinearOpMode {

    private RobotMecanum robot;
    SampleMecanumDrive drive;
    SelectLinear sl = new SelectLinear(this);
    int detectionWaitTime = 650;
    long startTime;
    long currentTime;
    HSVPipeline detector;
    OpenCvWebcam webcam;
    EasyOpenCVExample.RingDeterminationPipeline pipeline;
    boolean Blue = true;
    propLocation location = propLocation.Middle;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry telems;
    float xPurpleDump, yPurpleDump, xYellowDump, yYellowDump, xPark, yPark, xIntake;
    TrajectorySequence test, dumpPurplePixel, extraForPurple, dumpYellowPixel1, dumpYellowPixel2, dumpYellowPixel3, park, goToIntake;
    Pose2d start = new Pose2d();

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            telems = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);
            robot = new RobotMecanum(this, true, true);
            drive = new SampleMecanumDrive(hardwareMap);
            WaitLinear lp = new WaitLinear(this);
            initCamera(); // initializes camera and sets up pipeline for team shipping element detection

            Blue = sl.selectAlliance();

            this.detector = new HSVPipeline();
            //this.detector.useDefaults();
            webcam.setPipeline(detector);
            //detector.isLeft(Top);

            try {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            } catch (Exception e) {
                try {
                    this.detector = new HSVPipeline();
                    //this.detector.useDefaults();
                    webcam.setPipeline(detector);
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                } catch (Exception f) {
                    telemetry.addLine("Error");
                    telemetry.update();
                    location = propLocation.Middle;
                }
            }

            telemetry.update();

            long time1 = System.currentTimeMillis();
            currentTime = System.currentTimeMillis();
            while (currentTime - time1 < detectionWaitTime) {
                location = detector.getLocation(!Blue, true);
                currentTime = System.currentTimeMillis();
            }

            //detector.reset();
            telemetry.addData("Prop Location", location);
            telemetry.addData("Blue?", Blue);
            telemetry.update();

            if (isStopRequested()) {
                return;
            }

            waitForStart();

            /*test = drive.actionBuilder(drive.pose)
                    .lineToY(10)
                    .build();*/


            if (opModeIsActive()) {
                robot.clearBulkCache();
                telemetry.addLine("running");
                telemetry.update();
                RobotLog.ii(RobotConstants.TAG_R, "location" + location);

                startTime = System.currentTimeMillis();
                time1 = System.currentTimeMillis();
                currentTime = System.currentTimeMillis();
                while (currentTime - time1 < detectionWaitTime) {
                    location = detector.getLocation(!Blue, true);
                    currentTime = System.currentTimeMillis();
                }

                robot.hang.setIn();


                robot.vSlides.reset(robot.vSlides.vSlides);

                //detector.reset();
                telemetry.addData("Blue alliance", Blue);
                telemetry.addData("Prop location", location);
                telemetry.update();
                yYellowDump = Blue? -90: 90;
                xIntake = Blue?-27:-27.5f;
                if(location==propLocation.Middle){
                    xPurpleDump = Blue? -29: -29;
                    yPurpleDump = Blue? -2: 1f;//-2: -0.5f;
                    xYellowDump = Blue? -23: -22;//-26: -26;
                    //yYellowDump = Blue? -36: 36;
                }
                else if(location==propLocation.Left){
                    xPurpleDump = Blue? -26: -26;
                    yPurpleDump = Blue? -2f: -5;//9.5f;
                    xYellowDump = Blue?  -18: -25f;//-18: -26.5f;//-18;
                    //yYellowDump = Blue? -36: 36;
                }
                else{ //right
                    xPurpleDump = Blue? -26: -26;
                    yPurpleDump = Blue? 5.5f: 0f;//-4;
                    xYellowDump = Blue? -27f: -13.75f;//-26f: -20f;//-36;
                    //yYellowDump = Blue? -36: 36;
                }

                xPark = -40;
                yPark = yYellowDump+5;

                //initialize trajectories
                dumpPurplePixel = drive.trajectorySequenceBuilder(start)
                        //.lineTo(new Vector2d(50, 0))
                        .lineToLinearHeading(new Pose2d(xPurpleDump, yPurpleDump, Math.toRadians(Blue? 90 : -90)))
                        .build();
                extraForPurple = drive.trajectorySequenceBuilder(dumpPurplePixel.end())
                        .lineTo(new Vector2d(xPurpleDump, Blue? yPurpleDump-15 : yPurpleDump+18))
                        .build();
                goToIntake = drive.trajectorySequenceBuilder(dumpPurplePixel.end())
                        .lineToLinearHeading(new Pose2d(xIntake, Blue? 15: -17, Math.toRadians(Blue? 90:-90)))
                        .build();
                dumpYellowPixel1 = drive.trajectorySequenceBuilder(goToIntake.end())
                        .lineToLinearHeading(new Pose2d(Blue? xIntake-23 :xIntake-23, Blue ? 17: -17, Math.toRadians(Blue? 90:-90)))
                        .build();
                dumpYellowPixel2 = drive.trajectorySequenceBuilder(dumpYellowPixel1.end())
                        .lineTo(new Vector2d(Blue? xIntake-21 :xIntake-21,Blue? -48:48))
                        .addSpatialMarker(new Vector2d(xIntake, Blue? 10:-10), () -> {
                            robot.intakeSmallTilt.setTransfer();
                        })
                        .addSpatialMarker(new Vector2d(xIntake, Blue? 7:-7), () -> {
                            robot.intakeDoor.setOpen();
                            robot.depoDoor.setOpen2();
                            robot.intake.in();
                        })
                        .build();
                dumpYellowPixel3 = drive.trajectorySequenceBuilder(dumpYellowPixel2.end())
                        .lineToLinearHeading(new Pose2d(xYellowDump, yYellowDump, Math.toRadians(Blue? 90 : -90)))
                        .addSpatialMarker(new Vector2d(xIntake, Blue? -35:36), () -> {
                            robot.depoTilt.setOut();
                        })
                        .build();
                /*dumpYellowPixel1 = drive.trajectorySequenceBuilder(goToIntake.end())
                        .lineToLinearHeading(new Pose2d(xIntake, Blue? -55:55, Math.toRadians(Blue? 90 : -90)))
                        .addSpatialMarker(new Vector2d(xIntake, Blue? -50:50), () -> {
                            robot.intakeSmallTilt.setOut();
                        })
                        .addSpatialMarker(new Vector2d(xIntake, Blue? -40:40), () -> {
                            robot.intakeBigTilt.setTransfer();
                        })
                        .addSpatialMarker(new Vector2d(xIntake, Blue? -30:30), () -> {
                            robot.intakeSmallTilt.setTransfer();
                        })
                        .addSpatialMarker(new Vector2d(xIntake, Blue? -10:10), () -> {
                            robot.intakeDoor.setOpen();
                            robot.intake.off();
                        })
                        .build();
                dumpYellowPixel2 = drive.trajectorySequenceBuilder(dumpYellowPixel1.end())
                        .lineToLinearHeading(new Pose2d(xYellowDump, yYellowDump, Math.toRadians(Blue? 90 : -90)))
                        .build();*/
                park = drive.trajectorySequenceBuilder(dumpYellowPixel2.end())
                        .lineTo(new Vector2d(xYellowDump, Blue? yYellowDump+5 : yYellowDump-5))
                        .build();

                /*robot.intakeDoor.setOpen();
                lp.waitMillis(500);

                telemetry.addLine("before stopSteaming");
                telemetry.update();
                lp.waitMillis(3000);*/

                webcam.stopStreaming();
                webcam.closeCameraDevice();

                telemetry.addLine("before AutoBody");
                telemetry.update();

                AutoBody(lp, Blue);
            } else {
                telemetry.addLine("Not active");
                telemetry.update();
            }

        } catch (Exception e) {
            RobotLog.e("Overcharged: Autonomous Failed: ", e.getMessage());
        } finally {
            // shut down
            if (robot != null) {
                robot.close();
            }
        }
    }

    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new EasyOpenCVExample.RingDeterminationPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDevice();
        webcam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
    }

    public void AutoBody(WaitLinear lp, boolean Blue) throws InterruptedException {
        RobotLog.ii(TAG_SL, "started");
        telemetry.addLine("running auto body");
        telemetry.update();
        RobotLog.ii(RobotConstants.TAG_R, "running auto body");

        robot.depoDoor.setClosed();
        robot.intakeDoor.setClosed();

        drive.followTrajectorySequence(dumpPurplePixel);

        if((location == propLocation.Right && !Blue) || (location == propLocation.Left && Blue))
            drive.followTrajectorySequence(extraForPurple);

        //robot.leftPixel.setDump();
        //lp.waitMillis(500);
        RobotLog.ii(RobotConstants.TAG_R, "left pixel isBlue" + Blue + "dump" + robot.pixel.LEFT_DUMP);

        if(Blue) {
            float lPixelPos = robot.pixel.leftPixelDropper.getPosition();//153f;
            long dropperTime = System.currentTimeMillis();
            while (lPixelPos >= robot.pixel.LEFT_DUMP && System.currentTimeMillis() - dropperTime < 1000) {
                RobotLog.ii(RobotConstants.TAG_R, "left pixel pos" + lPixelPos + "dump" + robot.pixel.LEFT_DUMP);
                RobotLog.ii(RobotConstants.TAG_R, "moving left pixel");
                lPixelPos -= 3;
                robot.pixel.setLeftPosition(lPixelPos);
                lp.waitMillis(7);
            }
        }
        else {
            float rPixelPos = robot.pixel.rightPixelDropper.getPosition();//153f;
            long dropperTime = System.currentTimeMillis();
            while (rPixelPos <= robot.pixel.RIGHT_DUMP && System.currentTimeMillis() - dropperTime < 500) {//hSlidesOut >= hSlides.MIN+10) {
                rPixelPos += 3;
                robot.pixel.setRightPosition(rPixelPos);
                lp.waitMillis(7);
            }
        }

        robot.intake.in();
        robot.intakeSmallTilt.setOut();
        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FIFTH);
        //robot.intakeSmallTilt.setPosition(robot.intakeSmallTilt.FIFTHP);
        drive.followTrajectorySequence(goToIntake);
        lp.waitMillis(500);

        robot.intake.off();
        robot.intakeBigTilt.setTransfer();
        robot.intakeSmallTilt.setTransfer();
        drive.followTrajectorySequence(dumpYellowPixel1);
        robot.intakeSmallTilt.setOut();
        drive.followTrajectorySequence(dumpYellowPixel2);
        robot.depoDoor.setClosed();
        robot.intake.off();
        robot.vSlides.moveEncoderTo(robot.vSlides.autoLevel, 1);
        lp.waitMillis(250);
        robot.depoTilt.setOut();
        drive.followTrajectorySequence(dumpYellowPixel3);
        //robot.vSlides.moveEncoderTo(robot.vSlides.autoLevel, 1);
        //lp.waitMillis(600);

        //robot.depoTilt.setOut();
        //drive.followTrajectorySequence(dumpYellowPixel2);

        robot.depoDoor.setOpen2();
        lp.waitMillis(500);

        robot.vSlides.moveEncoderTo(robot.vSlides.autoLevel+150, 1);
        lp.waitMillis(500);

        //drive.followTrajectorySequence(park);
        robot.depoTilt.setIn();
        lp.waitMillis(500);

        robot.vSlides.vSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        long slideDownTime = System.currentTimeMillis();
        RobotLog.ii(RobotConstants.TAG_R, "reached bottom? " + robot.vSlides.switchSlideDown.isTouch() + " time elapsed " + (System.currentTimeMillis() - slideDownTime));
        while(!robot.vSlides.switchSlideDown.isTouch() && System.currentTimeMillis() - slideDownTime < 2000){
            RobotLog.ii(RobotConstants.TAG_R, "reached bottom? " + robot.vSlides.switchSlideDown.isTouch() + " power " + robot.vSlides.vSlides.getPower() + " time elapsed " + (System.currentTimeMillis() - slideDownTime));
            robot.vSlides.moveToBottom();
        }
        robot.vSlides.setPower(0);
        robot.vSlides.forcestop();
        robot.vSlides.reset(robot.vSlides.vSlides);

        //lowerSlidesThread(lp);
        lp.waitMillis(30000-System.currentTimeMillis()+startTime);
    }

    public void lowerSlidesThread(WaitLinear lp) { // asynchronously start raising the slides
        Runnable lowerSlidesThread = new vSlidesThread(0.6f, false, lp, this, robot);
        Thread thread = new Thread(lowerSlidesThread);
        thread.start();
    }
}