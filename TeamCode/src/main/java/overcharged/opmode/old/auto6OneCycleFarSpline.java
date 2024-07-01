package overcharged.opmode.old;


import static overcharged.config.RobotConstants.TAG_SL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import overcharged.components.RobotMecanum;
import overcharged.components.hslides;
import overcharged.components.intakeBigTilt;
import overcharged.components.intakeSmallTilt;
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
@Autonomous(name="auto6OneCycleFar2.0")
public class auto6OneCycleFarSpline extends LinearOpMode {

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
    float xPurpleDump, yPurpleDump, xYellowDump, yYellowDump, xPark, yPark, xIntake;
    TrajectorySequence test, dumpPurplePixel, extraForPurple, dumpYellowPixel1, dumpYellowPixel2,
            dumpYellowPixel3, extraPush, park, goToIntake, cycleIntake1, cycleIntake2, additionalCycleDump, additionalCycleDump2, extraPush2;
    Pose2d start = new Pose2d();

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            telems = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);
            robot = new RobotMecanum(this, true, true);
            drive = new SampleMecanumDrive(hardwareMap);
            WaitLinear lp = new WaitLinear(this);
            initCamera(); // initializes camera and sets up pipeline for team shipping element detection

            //initialize trajectories
            /*dumpPurplePixel = drive.trajectorySequenceBuilder(start)
                    //.lineTo(new Vector2d(50, 0))
                    .lineToLinearHeading(new Pose2d(xPurpleDump, yPurpleDump, Math.toRadians(Blue? 90 : -90)))
                    .build();
            extraForPurple = drive.trajectorySequenceBuilder(dumpPurplePixel.end())
                    .lineTo(new Vector2d(xPurpleDump, Blue? yPurpleDump-15 : yPurpleDump+18))
                    .build();
            goToIntake = drive.trajectorySequenceBuilder(dumpPurplePixel.end())
                    .lineToLinearHeading(new Pose2d(xIntake, Blue? 18: -17, Math.toRadians(Blue? 90:-90)))
                    .build();
            dumpYellowPixel1 = drive.trajectorySequenceBuilder(goToIntake.end())
                    .lineToLinearHeading(new Pose2d(Blue? xIntake-23 :xIntake-23, Blue ? 18: -18, Math.toRadians(Blue? 90:-90)))
                    .build();
            dumpYellowPixel2 = drive.trajectorySequenceBuilder(dumpYellowPixel1.end())
                    .lineTo(new Vector2d(Blue? xIntake-23 :xIntake-21,Blue? -48:48))
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
                    .addSpatialMarker(new Vector2d(xIntake, Blue? -35:35), () -> {
                        robot.depoTilt.setOut();
                    })
                    .build();
            cycleIntake1 = drive.trajectorySequenceBuilder(dumpYellowPixel3.end())
                    .lineToLinearHeading(new Pose2d(Blue? xIntake-23 :xIntake-21,Blue? -48:48, Math.toRadians(Blue? 90 : -90)))
                    .build();
            cycleIntake2 = drive.trajectorySequenceBuilder(cycleIntake1.end())
                    .lineToLinearHeading(new Pose2d(Blue? xIntake-23 :xIntake-19, Blue ? -15: 15, Math.toRadians(Blue? 90 : -95)))
                    .build();
            additionalCycleDump = drive.trajectorySequenceBuilder(cycleIntake2.end())
                    .lineTo(new Vector2d(Blue? xIntake-23 :xIntake-21,Blue? -48:48))
                    .build();*/

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
                yYellowDump = Blue? -93f: 93f;
                xIntake = -26;
                if(location==propLocation.Middle){
                    xPurpleDump = Blue? -29: -29;
                    yPurpleDump = Blue? -2: 1f;
                    xYellowDump = Blue? -23: -21;
                    //yYellowDump = Blue? -36: 36;
                }
                else if(location==propLocation.Left){
                    xPurpleDump = Blue? -26: -26;
                    yPurpleDump = Blue? -2f: -5;//9.5f;
                    xYellowDump = Blue? -18: -25f;//-18;
                    //yYellowDump = Blue? -36: 36;
                }
                else{ //right
                    xPurpleDump = Blue? -26: -25;
                    yPurpleDump = Blue? 5.5f: 0f;//-4;
                    xYellowDump = Blue? -27f: -17f;//-36;
                    //yYellowDump = Blue? -36: 36;
                }

                xPark = -40;
                yPark = yYellowDump+5;

                //initialize trajectories
                /*Trajectory traj3 = drive.trajectoryBuilder(start)
                        .lineTo(new Vector2d(-20,0),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .splineToConstantHeading(new Vector2d(-30,10),0,
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();
                TrajectorySequence test1 = drive.trajectorySequenceBuilder(start)
                        .lineTo(new Vector2d(-20,0),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .splineToConstantHeading(new Vector2d(-30,10),Math.toRadians(180),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();*/
                dumpPurplePixel = drive.trajectorySequenceBuilder(start)
                        //.lineTo(new Vector2d(50, 0))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(75, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                        .lineToLinearHeading(new Pose2d(xPurpleDump, yPurpleDump, Math.toRadians(Blue? 90 : -90)))
                        .build();
                extraForPurple = drive.trajectorySequenceBuilder(dumpPurplePixel.end())
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(75, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                        .lineTo(new Vector2d(xPurpleDump, Blue? yPurpleDump-16 : yPurpleDump+19))
                        .build();
                goToIntake = drive.trajectorySequenceBuilder(dumpPurplePixel.end())
                        .lineToLinearHeading(new Pose2d(xIntake, Blue? 18: -17, Math.toRadians(Blue? 90:-90)))
                        .build();
                dumpYellowPixel1 = drive.trajectorySequenceBuilder(goToIntake.end())
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(75, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                        .lineToLinearHeading(new Pose2d(Blue? xIntake-23 :xIntake-23, Blue ? 17: -17, Math.toRadians(Blue? 90:-90)))
                        .addSpatialMarker(new Vector2d(Blue? xIntake-4 :xIntake-4, Blue? 10:-10), () -> {
                            robot.intake.out();
                            //robot.intakeSmallTilt.setPosition(intakeSmallTilt.DUMP_EXTRA);
                            //robot.intakeBigTilt.setPosition(intakeBigTilt.DUMP_EXTRA);
                        })
                        .addSpatialMarker(new Vector2d(Blue? xIntake-5 :xIntake-5, Blue? 10:-10), () -> {
                            robot.intake.off();
                            //robot.intakeSmallTilt.setPosition(intakeSmallTilt.DUMP_EXTRA);
                            //robot.intakeBigTilt.setPosition(intakeBigTilt.DUMP_EXTRA);
                        })
                        .addSpatialMarker(new Vector2d(Blue? xIntake-10 :xIntake-10, Blue? 10:-10), () -> {
                            robot.intake.in();
                            //robot.intakeSmallTilt.setPosition(intakeSmallTilt.DUMP_EXTRA);
                            //robot.intakeBigTilt.setPosition(intakeBigTilt.DUMP_EXTRA);
                        })
                        .build();
                dumpYellowPixel2 = drive.trajectorySequenceBuilder(dumpYellowPixel1.end())
                        .lineTo(new Vector2d(Blue? xIntake-21 :xIntake-21,Blue? -63:63),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        //.splineTo(new Vector2d(Blue? xIntake-21 :xIntake-21,Blue? -63:63), Math.toRadians(Blue? 90 : -90))
                        .addSpatialMarker(new Vector2d(xIntake, Blue? 7:-7), () -> {
                            //robot.vSlides.moveEncoderTo(100, 1);
                            robot.hslides.moveEncoderTo(hslides.START,1);
                            robot.intakeDoor.setOpen();
                            robot.depoDoor.setOpen2();
                            robot.intake.in();
                        })
                        .addSpatialMarker(new Vector2d(Blue? xIntake-21 :xIntake-21, Blue? -55:55), () -> {
                            robot.intakeDoor.setClosed();
                            robot.depoDoor.setClosed();
                            robot.intake.off();
                            //robot.vSlides.moveEncoderTo(DropHeight ? robot.vSlides.autoLevel : robot.vSlides.autoLevel+50, 1);
                        })
                        .splineToConstantHeading(new Vector2d(xYellowDump, Blue? yYellowDump+13 : yYellowDump-13), Math.toRadians(Blue? 90 : -90),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        //.splineTo(new Vector2d(xYellowDump, Blue? yYellowDump+5 : yYellowDump-5), Math.toRadians(Blue? 90 : -90))
                        /*.addSpatialMarker(new Vector2d(xYellowDump, Blue? yYellowDump+10 : yYellowDump-10), () -> {
                            robot.depoTilt.setOut();
                        })*/
                        .build();

                /*dumpYellowPixel3 = drive.trajectorySequenceBuilder(dumpYellowPixel2.end())
                        .lineToLinearHeading(new Pose2d(xYellowDump, Blue? yYellowDump+5 : yYellowDump-5, Math.toRadians(Blue? 90 : -90)))
                        .addSpatialMarker(new Vector2d(xIntake, Blue? -64:60), () -> {
                            robot.depoTilt.setOut();
                        })
                        .build();*/
                telemetry.addLine("AFTER init");
                telemetry.update();
                extraPush = drive.trajectorySequenceBuilder(dumpYellowPixel2.end())
                        .lineToLinearHeading(new Pose2d(xYellowDump, yYellowDump, Math.toRadians(Blue? 90 : -90)))
                        .addSpatialMarker(new Vector2d(xYellowDump, Blue? yYellowDump+8 : yYellowDump-8), () -> {
                            robot.depoTilt.setOut();
                        })
                        .build();
                cycleIntake1 = drive.trajectorySequenceBuilder(extraPush.end())
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(85, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                        .lineToLinearHeading(new Pose2d(Blue? xIntake-22.5 :xIntake-22.5,Blue? -63:63, Math.toRadians(Blue? 90 : -90)))
                        .addSpatialMarker(new Vector2d(Blue? xIntake-22.5 :xIntake-22,Blue? -55:55), () -> {
                            slidesDown(lp);
                        })
                        .build();
                cycleIntake2 = drive.trajectorySequenceBuilder(cycleIntake1.end())
                        .lineToLinearHeading(new Pose2d(Blue? xIntake-25 :xIntake-22, Blue ? -10: 10, Math.toRadians(Blue? 94  : -90)))
                        .build();
                additionalCycleDump = drive.trajectorySequenceBuilder(cycleIntake2.end())
                        .lineTo(new Vector2d(Blue? xIntake-24 :xIntake-22,Blue? -63:63),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        //.lineTo(new Vector2d(Blue? xIntake-24 :xIntake-22,Blue? -63:63))
                        .addSpatialMarker(new Vector2d(Blue? xIntake-25 :xIntake-22, Blue? -13:13), () -> {
                            robot.intake.out();
                        })
                        .addSpatialMarker(new Vector2d(Blue? xIntake-25 :xIntake-22, Blue? -13.5:13.5), () -> {
                            robot.intake.off();
                        })
                        .addSpatialMarker(new Vector2d(Blue? xIntake-25 :xIntake-22, Blue? -17:17), () -> {
                            robot.intake.in();
                        })
                        .addSpatialMarker(new Vector2d(Blue? xIntake-25 :xIntake-22, Blue? -30:30), () -> {
                            robot.intake.in();
                            robot.intakeDoor.setOpen();
                            robot.depoDoor.setOpen2();
                        })
                        .addSpatialMarker(new Vector2d(Blue? xIntake-25 :xIntake-22, Blue? -60:60), () -> {
                            robot.intakeDoor.setClosed();
                            robot.depoDoor.setClosed();
                            robot.intake.off();
                            //robot.vSlides.moveEncoderTo(robot.vSlides.level1, 1);
                        })
                        .splineToConstantHeading(new Vector2d(Blue? -23: -18, Blue? yYellowDump+13 : yYellowDump-13), Math.toRadians(Blue? 90 : -90),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        /*.addSpatialMarker(new Vector2d(xIntake, Blue? -64:60), () -> {
                            robot.depoTilt.setOut();
                        })*/
                        .build();
                /*additionalCycleDump2 = drive.trajectorySequenceBuilder(additionalCycleDump.end())
                        .lineToLinearHeading(new Pose2d(Blue? -23: -18, Blue? yYellowDump+5 : yYellowDump-5, Math.toRadians(Blue? 90 : -90)))
                        .addSpatialMarker(new Vector2d(xIntake, Blue? -64:60), () -> {
                            robot.depoTilt.setOut();
                        })
                        .build();*/
                extraPush2 = drive.trajectorySequenceBuilder(additionalCycleDump.end())
                        .lineToLinearHeading(new Pose2d(xYellowDump, yYellowDump, Math.toRadians(Blue? 90 : -90)))
                        .addSpatialMarker(new Vector2d(xYellowDump, Blue? yYellowDump+8 : yYellowDump-8), () -> {
                            robot.depoTilt.setOut();
                        })
                        .build();

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
        lp.waitMillis(75);


        robot.hslides.moveEncoderTo(hslides.START,-1);
        robot.intake.off();
        robot.intakeBigTilt.setTransfer();
        robot.intakeSmallTilt.setTransfer();
        drive.followTrajectorySequence(dumpYellowPixel1);
        robot.intakeSmallTilt.setOut();
        drive.followTrajectorySequence(dumpYellowPixel2);
        /*robot.intakeDoor.setClosed();
        robot.depoDoor.setClosed();
        robot.intake.off();
        robot.vSlides.moveEncoderTo(DropHeight ? robot.vSlides.autoLevel : robot.vSlides.autoLevel+50, 1);*/
        //drive.followTrajectorySequence(dumpYellowPixel3);
        robot.vSlides.moveEncoderTo(DropHeight ? robot.vSlides.autoLevel : robot.vSlides.autoLevel+50, 1);
        //lp.waitMillis(500);
        drive.followTrajectorySequence(extraPush);

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
            robot.vSlides.moveEncoderTo(robot.vSlides.level1, 1);
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
            //lowerSlidesThread(lp);
        }

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