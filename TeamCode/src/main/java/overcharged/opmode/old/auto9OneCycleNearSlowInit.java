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
import overcharged.components.hslides;
import overcharged.components.propLocation;
import overcharged.config.RobotConstants;
import overcharged.drive.DriveConstants;
import overcharged.linear.util.SelectLinear;
import overcharged.linear.util.WaitLinear;
import overcharged.drive.SampleMecanumDrive;
import overcharged.opmode.hSlidesThread;
import overcharged.opmode.vSlidesThread;
import overcharged.test.EasyOpenCVExample;
import overcharged.test.HSVPipeline;
import overcharged.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(name="auto9OneCycleNearSlowInit")
public class auto9OneCycleNearSlowInit extends LinearOpMode {

    private RobotMecanum robot;
    SampleMecanumDrive drive;
    SelectLinear sl = new SelectLinear(this);
    int detectionWaitTime = 650;
    long startTime;
    long currentTime;
    double waitTime = 0;
    HSVPipeline detector;
    OpenCvWebcam webcam;
    EasyOpenCVExample.RingDeterminationPipeline pipeline;
    boolean Blue = true;
    boolean DropHeight = true; //true = low, false = high drop
    propLocation location = propLocation.Middle;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry telems;
    float yYellowDump, xIntake, xMPurpleDump, yMPurpleDump, xMYellowDump, xLPurpleDump, yLPurpleDump,
            xLYellowDump, xRPurpleDump, yRPurpleDump, xRYellowDump;
    TrajectorySequence dumpMPurplePixel, dumpLPurplePixel, dumpRPurplePixel, extraForPurple, goToIntake,
            bridgeGoToIntake, dumpYellowPixel1, dumpMYellowPixel2, dumpLYellowPixel2, dumpRYellowPixel2,
            extraMBluePush, extraLBluePush, extraRBluePush, extraMPush, extraLPush, extraRPush, cycleIntake1, cycleIntake2, additionalCycleDump, extraPush2, park;
    //float xPurpleDump, yPurpleDump, xYellowDump, xPark, yPark;
    //TrajectorySequence test, dumpPurplePixel, extraForPurple, dumpYellowPixel1, dumpYellowPixel2,
    //dumpYellowPixel3, extraPush, park, goToIntake, cycleIntake1, cycleIntake2, additionalCycleDump, additionalCycleDump2, extraPush2;
    Pose2d start = new Pose2d();

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            telems = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);
            robot = new RobotMecanum(this, true, true);
            drive = new SampleMecanumDrive(hardwareMap);
            WaitLinear lp = new WaitLinear(this);
            initCamera(); // initializes camera and sets up pipeline for team shipping element detection
            robot.hang.setIn();

            //initialize trajectories

            Blue = sl.selectAlliance();
            DropHeight = sl.selectDropHeight(); //true = low, false = high drop
            waitTime = sl.adjustDelay();

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

                yYellowDump = Blue? -35: 36;
                xIntake = Blue ? -27.5f : -26;

                //MIDDLE
                xMPurpleDump = Blue? -29: -29;
                yMPurpleDump = Blue? -2: 1f;
                xMYellowDump = Blue? -25: -25;

                //LEFT
                xLPurpleDump = Blue? -26: -26;
                yLPurpleDump = Blue? -10.5f: -4;//9.5f;
                xLYellowDump = Blue? -18: -18;//-33;

                //RIGHT
                xRPurpleDump = Blue? -26: -25;
                yRPurpleDump = Blue? 4f: 10.5f;//-4;
                xRYellowDump = Blue? -34.5f: -34.5f;//-18;

                //initialize trajectories
                telemetry.addLine("STARTING trajectory init");
                telemetry.update();
                dumpMPurplePixel = drive.trajectorySequenceBuilder(start)
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(75, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                        .lineToLinearHeading(new Pose2d(xMPurpleDump, yMPurpleDump, Math.toRadians(Blue? 90 : -90)))
                        .build();

                dumpLPurplePixel = drive.trajectorySequenceBuilder(start)
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(75, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                        .lineToLinearHeading(new Pose2d(xLPurpleDump, yLPurpleDump, Math.toRadians(Blue? 90 : -90)))
                        .build();
                dumpRPurplePixel = drive.trajectorySequenceBuilder(start)
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(75, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                        .lineToLinearHeading(new Pose2d(xRPurpleDump, yRPurpleDump, Math.toRadians(Blue? 90 : -90)))
                        .build();
                extraForPurple = drive.trajectorySequenceBuilder(Blue? dumpLPurplePixel.end() : dumpRPurplePixel.end())
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(75, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                        .lineTo(new Vector2d(Blue? xLPurpleDump : xRPurpleDump, Blue? yLPurpleDump-16 : yRPurpleDump+19))
                        .build();

                dumpMYellowPixel2 = drive.trajectorySequenceBuilder(dumpMPurplePixel.end())
                        .lineToLinearHeading(new Pose2d(xMYellowDump,Blue? -yYellowDump:yYellowDump, Math.toRadians(Blue? 90 : -90)),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .addSpatialMarker(new Vector2d(xMYellowDump, Blue? -30:30), () -> {
                            robot.depoTilt.setOut();
                        })
                        .build();
                dumpLYellowPixel2 = drive.trajectorySequenceBuilder(Blue? dumpLPurplePixel.end() : extraForPurple.end())
                        .lineToLinearHeading(new Pose2d(xLYellowDump,Blue? -yYellowDump:yYellowDump, Math.toRadians(Blue? 90 : -90)),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .addSpatialMarker(new Vector2d(xLYellowDump, Blue? -30:30), () -> {
                            robot.depoTilt.setOut();
                        })
                        .build();
                dumpRYellowPixel2 = drive.trajectorySequenceBuilder(Blue? extraForPurple.end() : dumpRPurplePixel.end())
                        .lineToLinearHeading(new Pose2d(xRYellowDump,Blue? -yYellowDump:yYellowDump, Math.toRadians(Blue? 90 : -90)),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .addSpatialMarker(new Vector2d(xRYellowDump, Blue? -30:30), () -> {
                            robot.depoTilt.setOut();
                        })
                        .build();

                extraMPush = drive.trajectorySequenceBuilder(dumpMYellowPixel2.end())
                        .splineToLinearHeading(new Pose2d(xMYellowDump, Blue? -yYellowDump: yYellowDump), Math.toRadians(Blue? 90 : -90))
                        .build();
                telemetry.addLine("FIRST trajectory init");
                telemetry.update();
                extraLPush = drive.trajectorySequenceBuilder(dumpLYellowPixel2.end())
                        .splineToLinearHeading(new Pose2d(xLYellowDump, Blue? -yYellowDump: yYellowDump), Math.toRadians(Blue? 90 : -90))
                        .build();
                telemetry.addLine("NEXT trajectory init");
                telemetry.update();
                extraRPush = drive.trajectorySequenceBuilder(dumpRYellowPixel2.end())
                        .splineToLinearHeading(new Pose2d(xRYellowDump, Blue? -yYellowDump: yYellowDump), Math.toRadians(Blue? 90 : -90))
                        .build();
                cycleIntake1 = drive.trajectorySequenceBuilder(Blue? dumpRYellowPixel2.end() : dumpLYellowPixel2.end())
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(85, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                        .splineToConstantHeading(new Vector2d(Blue? xIntake-22.5 :xIntake-22.5,Blue? 20:-20), Math.toRadians(Blue? 90 : -90))
                        .addSpatialMarker(new Vector2d(Blue? xIntake-22.5 :xIntake-22,Blue? -0:0), () -> {
                            slidesDown(lp);
                        })
                        .build();
                cycleIntake2 = drive.trajectorySequenceBuilder(cycleIntake1.end())
                        .lineToLinearHeading(new Pose2d(Blue? xIntake-25 :xIntake-22, Blue ? 30: -30, Math.toRadians(Blue? 91  : -90)))
                        .build();
                telemetry.addLine("SECOND middle trajectory init");
                telemetry.update();
                additionalCycleDump = drive.trajectorySequenceBuilder(cycleIntake2.end())
                        .lineTo(new Vector2d(Blue? xIntake-24 :xIntake-22,Blue? 20:-20),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        //.lineTo(new Vector2d(Blue? xIntake-24 :xIntake-22,Blue? -63:63))
                        .addSpatialMarker(new Vector2d(Blue? xIntake-25 :xIntake-22, Blue? 28:-28), () -> {
                            robot.intake.out();
                        })
                        .addSpatialMarker(new Vector2d(Blue? xIntake-25 :xIntake-22, Blue? 27.5:-27.5), () -> {
                            robot.intake.off();
                        })
                        .addSpatialMarker(new Vector2d(Blue? xIntake-25 :xIntake-22, Blue? 25:-25), () -> {
                            robot.intake.in();
                        })
                        .addSpatialMarker(new Vector2d(Blue? xIntake-25 :xIntake-22, Blue? 26:-26), () -> {
                            robot.intake.in();
                            robot.intakeDoor.setOpen();
                            robot.depoDoor.setOpen2();
                        })
                        .addSpatialMarker(new Vector2d(Blue? xIntake-25 :xIntake-22, Blue? 21:-21), () -> {
                            robot.intakeDoor.setClosed();
                            robot.depoDoor.setClosed();
                            robot.intake.off();
                            //robot.vSlides.moveEncoderTo(robot.vSlides.level1, 1);
                        })
                        .splineToConstantHeading(new Vector2d(xMYellowDump, Blue? -(yYellowDump-13) : yYellowDump-13), Math.toRadians(Blue? 90 : -90),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();
                telemetry.addLine("ALMOST DONE trajectory init");
                telemetry.update();
                extraPush2 = drive.trajectorySequenceBuilder(additionalCycleDump.end())
                        .lineToLinearHeading(new Pose2d(Blue? -25: -18, Blue? -yYellowDump : yYellowDump, Math.toRadians(Blue? 90 : -90)))
                        .addSpatialMarker(new Vector2d(Blue? -25: -18, Blue? -(yYellowDump-11) : yYellowDump-11), () -> {
                            robot.depoTilt.setOut();
                        })
                        .build();
                park = drive.trajectorySequenceBuilder(extraPush2.end())
                        .splineToConstantHeading(new Vector2d(0, Blue? yYellowDump-2:yYellowDump+2), Math.toRadians(Blue? 90 : -90))
                        .build();
                telemetry.addLine("After trajectory init");
                telemetry.update();

                webcam.stopStreaming();
                webcam.closeCameraDevice();

                telemetry.addLine("before AutoBody");
                telemetry.update();

                //drive.followTrajectory(traj3);
                //drive.followTrajectorySequence(test1);
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
        keepHSlidesIn(lp, true);

        lp.waitMillis(waitTime);

        robot.depoDoor.setClosed();
        robot.intakeDoor.setClosed();

        if(location == propLocation.Middle) { drive.followTrajectorySequence(dumpMPurplePixel);}
        else if(location == propLocation.Left) { drive.followTrajectorySequence(dumpLPurplePixel);}
        else { drive.followTrajectorySequence(dumpRPurplePixel); }

        if((location == propLocation.Right && Blue) || (location == propLocation.Left && !Blue))
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
                lp.waitMillis(9);
            }
        }
        else {
            float rPixelPos = robot.pixel.rightPixelDropper.getPosition();//153f;
            long dropperTime = System.currentTimeMillis();
            while (rPixelPos <= robot.pixel.RIGHT_DUMP && System.currentTimeMillis() - dropperTime < 500) {//hSlidesOut >= hSlides.MIN+10) {
                rPixelPos += 3;
                robot.pixel.setRightPosition(rPixelPos);
                lp.waitMillis(9);
            }
        }

        robot.vSlides.moveEncoderTo(robot.vSlides.autoLevel-40, 1);
        if(location == propLocation.Middle) { drive.followTrajectorySequence(dumpMYellowPixel2);}
        else if(location == propLocation.Left) { drive.followTrajectorySequence(dumpLYellowPixel2);}
        else { drive.followTrajectorySequence(dumpRYellowPixel2); }


        if (location == propLocation.Middle) {
            drive.followTrajectorySequence(extraMPush);
        } else if (location == propLocation.Left) {
            drive.followTrajectorySequence(extraLPush);
        } else {
            drive.followTrajectorySequence(extraRPush);
        }
        //robot.vSlides.moveEncoderTo(robot.vSlides.autoLevel, 1);
        //lp.waitMillis(600);

        //robot.depoTilt.setOut();
        //drive.followTrajectorySequence(dumpYellowPixel2);

        robot.depoDoor.setOpen2();
        lp.waitMillis(100);

        robot.vSlides.moveEncoderTo(robot.vSlides.level4, 1);
        /*robot.vSlides.vSlidesF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.vSlides.vSlidesB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.vSlides.vSlidesB.setPower(1);
        robot.vSlides.vSlidesF.setPower(1);*/
        lp.waitMillis(250);

        //drive.followTrajectorySequence(park);
        robot.depoTilt.setIn();
        lp.waitMillis(200);
        robot.intakeSmallTilt.setOut();
        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FIFTH);
        robot.intakeDoor.setClosed();
        //lowerSlidesThread(lp);

        if(waitTime < 2000) {
            drive.followTrajectorySequence(cycleIntake1);
            keepHSlidesIn(lp, false);
            robot.hslides.moveEncoderTo(1900, 1);
            robot.intake.in();

            drive.followTrajectorySequence(cycleIntake2);

            lp.waitMillis(75);
            robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FOURTH);
            lp.waitMillis(250);

            robot.hslides.moveEncoderTo(hslides.START,-1);
            keepHSlidesIn(lp, true);
            //robot.intake.off();
            robot.intakeDoor.setClosed();
            robot.intakeBigTilt.setTransfer();
            robot.intakeSmallTilt.setTransfer();
            drive.followTrajectorySequence(additionalCycleDump);
            robot.vSlides.moveEncoderTo(robot.vSlides.level1-40, 1);
            drive.followTrajectorySequence(extraPush2);
            /*robot.intakeDoor.setClosed();
            robot.depoDoor.setClosed();
            robot.intake.off();
            robot.vSlides.moveEncoderTo(robot.vSlides.level1, 1);*/
            //drive.followTrajectorySequence(additionalCycleDump2);

            robot.depoDoor.setOpen2();
            lp.waitMillis(100);

            robot.vSlides.moveEncoderTo(robot.vSlides.level4, 1);
            /*robot.vSlides.vSlidesB.setPower(1);
            robot.vSlides.vSlidesF.setPower(1);*/
            lp.waitMillis(250);
            robot.depoTilt.setIn();
            lp.waitMillis(250);

            slidesDown(lp);
            robot.hang.setIn();
            //lowerSlidesThread(lp);
        }

        drive.followTrajectorySequence(park);

        lp.waitMillis(30000-System.currentTimeMillis()+startTime);
    }

    public void slidesDown(WaitLinear lp){
        robot.vSlides.vSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(!robot.vSlides.slideReachedBottom()){
            robot.vSlides.down();
        }
        robot.vSlides.forcestop();
        robot.vSlides.vSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void lowerSlidesThread(WaitLinear lp) { // asynchronously start raising the slides
        Runnable lowerSlidesThread = new vSlidesThread(0.6f, false, lp, this, robot);
        Thread thread = new Thread(lowerSlidesThread);
        thread.start();
    }

    public void keepHSlidesIn(WaitLinear lp, boolean in) { // asynchronously start raising the slides
        Runnable keepHSlidesIn = new hSlidesThread(in, lp, this, robot);
        Thread thread = new Thread(keepHSlidesIn);
        thread.start();
    }
}