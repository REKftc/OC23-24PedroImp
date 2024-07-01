package overcharged.opmode;


import static overcharged.config.RobotConstants.TAG_SL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import overcharged.test.EasyOpenCVExample;
import overcharged.test.HSVPipeline;
import overcharged.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="auto10NoCycleFarWall")
public class auto10NoCycleFarWall extends LinearOpMode {

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
    boolean ParkingLocation = true; //true = wall, false = center
    boolean notWithRV = true;
    propLocation location = propLocation.Middle;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry telems;
    TrajectorySequence dumpMPurplePixel, dumpLPurplePixel, dumpRPurplePixel, extraForPurple, goToIntake,
            bridgeGoToIntake, dumpYellowPixel1, dumpMYellowPixel2, dumpLYellowPixel2, dumpRYellowPixel2,
            extraMPush, extraLPush, extraRPush, cycleIntake1, cycleIntake2, additionalCycleDump, extraPush2,
            turnCorrection1, turnCorrection2M, turnCorrection2L, turnCorrection2R,turnCorrection3M,
            turnCorrection3L, turnCorrection3R, wallPark, centerPark;
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
            //robot.depo.setBothClawsOpen();
            robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
            robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
            Blue = sl.selectAlliance();
            if(Blue){robot.pixel.setLeftIn();}
            else{robot.pixel.setRightIn();}
            DropHeight = sl.selectDropHeight(); //true = low, false = high drop
            ParkingLocation = sl.selctParkingLocation(); //true = wall, false = center
            waitTime = sl.adjustDelay();
            notWithRV = sl.selectRV();

            float yYellowDump = 91;//94f;
            float xIntake = Blue ? -27.5f : -26;

            //MIDDLE
            float xMPurpleDump = Blue? -29: -29;
            float yMPurpleDump = Blue? -2: 1f;
            float xMYellowDump = Blue? -26: -26f;//-25: -22;

            //LEFT
            float xLPurpleDump = Blue? -25: -26;
            float yLPurpleDump = Blue? -2f: -4.5f;//9.5f;
            float xLYellowDump = Blue? -22f: -33f;//-18;

            //RIGHT
            float xRPurpleDump = Blue? -26: -25;
            float yRPurpleDump = Blue? 4.5f: 0f;//-4;
            float xRYellowDump = Blue? -33.5f: -18.9f;//-28.5f: -19.5f;//-36;

            //initialize trajectories
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
            goToIntake = drive.trajectorySequenceBuilder(Blue? dumpRPurplePixel.end() : dumpLPurplePixel.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(xIntake, Blue? 20: -16.5, Math.toRadians(Blue? 90:-90)))
                    .build();
            bridgeGoToIntake = drive.trajectorySequenceBuilder(extraForPurple.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(xIntake, Blue? 20: -16.5, Math.toRadians(Blue? 90:-90)))
                    .build();
            dumpYellowPixel1 = drive.trajectorySequenceBuilder(goToIntake.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToConstantHeading(new Vector2d(Blue? -3.5:-3, Blue ? 17: -17))
                    .addSpatialMarker(new Vector2d(Blue? xIntake+4 :xIntake+4, Blue? 10:-10), () -> {
                        robot.intake.out();
                        //robot.intakeSmallTilt.setPosition(intakeSmallTilt.DUMP_EXTRA);
                        //robot.intakeBigTilt.setPosition(intakeBigTilt.DUMP_EXTRA);
                    })
                    .addSpatialMarker(new Vector2d(Blue? xIntake+4.4 :xIntake+4.4, Blue? 10:-10), () -> {
                        robot.intake.off();
                        //robot.intakeSmallTilt.setPosition(intakeSmallTilt.DUMP_EXTRA);
                        //robot.intakeBigTilt.setPosition(intakeBigTilt.DUMP_EXTRA);
                    })
                    .addSpatialMarker(new Vector2d(Blue? xIntake+10 :xIntake+10, Blue? 10:-10), () -> {
                        robot.intake.in();
                        //robot.intakeSmallTilt.setPosition(intakeSmallTilt.DUMP_EXTRA);
                        //robot.intakeBigTilt.setPosition(intakeBigTilt.DUMP_EXTRA);
                    })
                    .build();
            turnCorrection1 = drive.trajectorySequenceBuilder(new Pose2d(dumpYellowPixel1.end().getX()+1, Blue? dumpYellowPixel1.end().getY()+1: dumpYellowPixel1.end().getY()-1, Math.toRadians(Blue? 90 : -90)))
                    .lineToLinearHeading(new Pose2d(Blue? -3.5:-3, Blue ? 17: -17, Math.toRadians(Blue? 90 : -90)))
                    .build();
            dumpMYellowPixel2 = drive.trajectorySequenceBuilder(turnCorrection1.end())
                    .lineTo(new Vector2d(Blue? -3.5:-3,Blue? -63:63),
                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    //.splineTo(new Vector2d(Blue? xIntake-21 :xIntake-21,Blue? -63:63), Math.toRadians(Blue? 90 : -90))
                    .addSpatialMarker(new Vector2d(-2, Blue? 7:-7), () -> {
                        //robot.vSlides.moveEncoderTo(100, 1);
                        //robot.hslides.moveEncoderTo(hslides.START,1);
                        robot.intakeDoor.setOpen();
                        //robot.depoDoor.setOpen2();
                        //robot.depo.setBothClawsOpen();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        robot.intake.in();
                    })
                    .addSpatialMarker(new Vector2d(-2, Blue? -55:55), () -> {
                        //robot.intakeDoor.setClosed();
                        //robot.depoDoor.setClosed();
                        //robot.depo.setBothClawsClose();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                        robot.intake.off();
                        //robot.vSlides.moveEncoderTo(DropHeight ? robot.vSlides.autoLevel : robot.vSlides.autoLevel+50, 1);
                    })
                    .splineToConstantHeading(new Vector2d(xMYellowDump, Blue? -(yYellowDump-13) : yYellowDump-13), Math.toRadians(Blue? 90 : -90),
                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    /*.addSpatialMarker(new Vector2d(xMYellowDump, Blue? -(yYellowDump-14) : yYellowDump-14), () -> {
                        robot.vSlides.moveEncoderTo(robot.vSlides.autoLevel+100, 1);
                    })*/
                    .build();
            dumpLYellowPixel2 = drive.trajectorySequenceBuilder(dumpYellowPixel1.end())
                    .lineTo(new Vector2d(-3,Blue? -63:63),
                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    //.splineTo(new Vector2d(Blue? xIntake-21 :xIntake-21,Blue? -63:63), Math.toRadians(Blue? 90 : -90))
                    .addSpatialMarker(new Vector2d(-2, Blue? 7:-7), () -> {
                        //robot.vSlides.moveEncoderTo(100, 1);
                        //robot.hslides.moveEncoderTo(hslides.START,1);
                        robot.intakeDoor.setOpen();
                        //robot.depoDoor.setOpen2();
                        //robot.depo.setBothClawsOpen();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        robot.intake.in();
                    })
                    .addSpatialMarker(new Vector2d(-2, Blue? -55:55), () -> {
                        //robot.intakeDoor.setClosed();
                        //robot.depoDoor.setClosed();
                        //robot.BothClawsClose();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                        robot.intake.off();
                        //robot.vSlides.moveEncoderTo(DropHeight ? robot.vSlides.autoLevel : robot.vSlides.autoLevel+50, 1);
                    })
                    .splineToConstantHeading(new Vector2d(xLYellowDump, Blue? -(yYellowDump-13) : yYellowDump-13), Math.toRadians(Blue? 90 : -90),
                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    /*.addSpatialMarker(new Vector2d(xLYellowDump, Blue? -(yYellowDump-14) : yYellowDump-14), () -> {
                        robot.vSlides.moveEncoderTo(robot.vSlides.autoLevel+100, 1);
                    })*/
                    .build();
            dumpRYellowPixel2 = drive.trajectorySequenceBuilder(dumpYellowPixel1.end())
                    .lineTo(new Vector2d(-3,Blue? -63:63),
                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    //.splineTo(new Vector2d(Blue? xIntake-21 :xIntake-21,Blue? -63:63), Math.toRadians(Blue? 90 : -90))
                    .addSpatialMarker(new Vector2d(-2, Blue? 7:-7), () -> {
                        //robot.vSlides.moveEncoderTo(100, 1);
                        //robot.hslides.moveEncoderTo(hslides.START,1);
                        robot.intakeDoor.setOpen();
                        //robot.depoDoor.setOpen2();
                        //robot.depo.setBothClawsOpen();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        robot.intake.in();
                    })
                    .addSpatialMarker(new Vector2d(-2, Blue? -55:55), () -> {
                        //robot.intakeDoor.setClosed();
                        //robot.depoDoor.setClosed();
                        //robot.depo.setBothClawsClose();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                        robot.intake.off();
                        //robot.vSlides.moveEncoderTo(DropHeight ? robot.vSlides.autoLevel : robot.vSlides.autoLevel+50, 1);
                    })
                    .splineToConstantHeading(new Vector2d(xRYellowDump, Blue? -(yYellowDump-13) : yYellowDump-13), Math.toRadians(Blue? 90 : -90),
                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    /*.addSpatialMarker(new Vector2d(xRYellowDump, Blue? -(yYellowDump-14) : yYellowDump-14), () -> {
                        robot.vSlides.moveEncoderTo(robot.vSlides.autoLevel+100, 1);
                    })*/
                    .build();
            turnCorrection2M = drive.trajectorySequenceBuilder(new Pose2d(dumpMYellowPixel2.end().getX()+1, Blue? dumpMYellowPixel2.end().getY()+1: dumpMYellowPixel2.end().getY()-1, Math.toRadians(Blue? 90 : -90)))
                    .lineToLinearHeading(new Pose2d(xMYellowDump, Blue? -(yYellowDump-13) : yYellowDump-13, Math.toRadians(Blue? 90 : -90)))
                    .build();
            turnCorrection2L = drive.trajectorySequenceBuilder(new Pose2d(dumpLYellowPixel2.end().getX()+1, Blue? dumpLYellowPixel2.end().getY()+1: dumpLYellowPixel2.end().getY()-1, Math.toRadians(Blue? 90 : -90)))
                    .lineToLinearHeading(new Pose2d(xLYellowDump, Blue? -(yYellowDump-13) : yYellowDump-13, Math.toRadians(Blue? 90 : -90)))
                    .build();
            turnCorrection2R = drive.trajectorySequenceBuilder(new Pose2d(dumpRYellowPixel2.end().getX()+1, Blue? dumpRYellowPixel2.end().getY()+1: dumpRYellowPixel2.end().getY()-1, Math.toRadians(Blue? 90 : -90)))
                    .lineToLinearHeading(new Pose2d(xRYellowDump, Blue? -(yYellowDump-13) : yYellowDump-13, Math.toRadians(Blue? 90 : -90)))
                    .build();
            extraMPush = drive.trajectorySequenceBuilder(turnCorrection2M.end())
                    .lineToLinearHeading(new Pose2d(xMYellowDump, Blue? -yYellowDump: yYellowDump, Math.toRadians(Blue? 90 : -90)))
                    /*.addSpatialMarker(new Vector2d(xMYellowDump, Blue? -(yYellowDump-7) : yYellowDump-7), () -> {
                        //robot.depo.setArmPos(robot.depo.ARM_OUT);
                        robot.depo.setDepoOutVert();
                    })*/
                    .addSpatialMarker(new Vector2d(xMYellowDump, Blue? -(yYellowDump-12) : yYellowDump-12), () -> {
                        //robot.depo.setWristPos(robot.depo.WRIST_FLAT);
                        //robot.depo.setDepoOutOppVert();
                        robot.depo.setArmPos(robot.depo.ARM_OUT);
                        robot.depo.setWristPos(robot.depo.WRIST_OPP_VERT);

                    })
                    .build();
            extraLPush = drive.trajectorySequenceBuilder(turnCorrection2L.end())
                    .lineToLinearHeading(new Pose2d(xLYellowDump, Blue? -yYellowDump: yYellowDump, Math.toRadians(Blue? 90 : -90)))
                    /*.addSpatialMarker(new Vector2d(xMYellowDump, Blue? -(yYellowDump-7) : yYellowDump-7), () -> {
                        //robot.depo.setArmPos(robot.depo.ARM_OUT);
                        robot.depo.setDepoOutVert();
                    })*/
                    .addSpatialMarker(new Vector2d(xMYellowDump, Blue? -(yYellowDump-12) : yYellowDump-12), () -> {
                        //robot.depo.setWristPos(robot.depo.WRIST_FLAT);
                        //robot.depo.setDepoOutOppVert();
                        robot.depo.setArmPos(robot.depo.ARM_OUT);
                        robot.depo.setWristPos(robot.depo.WRIST_OPP_VERT);
                    })
                    .build();
            extraRPush = drive.trajectorySequenceBuilder(turnCorrection2R.end())
                    .lineToLinearHeading(new Pose2d(xRYellowDump, Blue? -yYellowDump: yYellowDump, Math.toRadians(Blue? 90 : -90)))
                    /*.addSpatialMarker(new Vector2d(xMYellowDump, Blue? -(yYellowDump-7) : yYellowDump-7), () -> {
                        //robot.depo.setArmPos(robot.depo.ARM_OUT);
                        robot.depo.setDepoOutVert();
                    })*/
                    .addSpatialMarker(new Vector2d(xMYellowDump, Blue? -(yYellowDump-12) : yYellowDump-12), () -> {
                        //robot.depo.setWristPos(robot.depo.WRIST_FLAT);
                        //robot.depo.setDepoOutOppVert();
                        robot.depo.setArmPos(robot.depo.ARM_OUT);
                        robot.depo.setWristPos(robot.depo.WRIST_OPP_VERT);
                    })
                    .build();
            wallPark = drive.trajectorySequenceBuilder(Blue? new Pose2d(dumpLYellowPixel2.end().getX(), dumpLYellowPixel2.end().getY()+11, Math.toRadians(90)) : new Pose2d(dumpRYellowPixel2.end().getX(), dumpRYellowPixel2.end().getY()-11, Math.toRadians(-90)))
                    .lineToConstantHeading(new Vector2d(-2, Blue? -(yYellowDump-4):yYellowDump-4))
                    .addSpatialMarker(new Vector2d(-6, Blue? yYellowDump:yYellowDump), () -> {
                        //robot.depo.setBothClawsClose();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                        //robot.depo.setDepoIn();
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_IN);
                    })
                    .addSpatialMarker(new Vector2d(-3, Blue? yYellowDump+2:yYellowDump-2), () -> {
                        //slidesDown(lp);
                    })
                    .build();
            centerPark = drive.trajectorySequenceBuilder(!Blue? new Pose2d(dumpLYellowPixel2.end().getX(), dumpLYellowPixel2.end().getY()+11, Math.toRadians(90)) : new Pose2d(dumpRYellowPixel2.end().getX(), dumpRYellowPixel2.end().getY()-11, Math.toRadians(-90)))
                    .lineToConstantHeading(new Vector2d(-60, Blue? -(yYellowDump):yYellowDump))
                    .addSpatialMarker(new Vector2d(-25, Blue? yYellowDump:yYellowDump), () -> {
                        // robot.depo.setDepoIn();
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_IN);
                    })
                    .addSpatialMarker(new Vector2d(-40, Blue? yYellowDump+2:yYellowDump-2), () -> {
                        slidesDown(lp);
                    })
                    .build();

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
            telemetry.addData("Wait time", waitTime);
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

                //robot.hang.setLeftIn();
                //robot.hang.setRightIn();
                robot.hang.setIn();
                //robot.vSlides.reset(robot.vSlides.vSlidesB);
                //robot.vSlides.reset(robot.vSlides.vSlidesF);
                robot.vSlides.reset(robot.vSlides.vSlides);

                //detector.reset();
                telemetry.addData("Blue alliance", Blue);
                telemetry.addData("Prop location", location);
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
        if(!notWithRV){
            if(Blue) {
                if (location == propLocation.Middle) {
                    lp.waitMillis(7300);
                    ;
                } else if (location == propLocation.Left) {
                    lp.waitMillis(6500);
                } else {
                    lp.waitMillis(8000);
                }
            } else{
                if (location == propLocation.Middle) {
                    lp.waitMillis(7300);
                    ;
                } else if (location == propLocation.Left) {
                    lp.waitMillis(8000);
                } else {
                    lp.waitMillis(6500);
                }
            }
        }

        //robot.depoDoor.setClosed();
        //robot.depo.setBothClawsClose();
        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
        robot.intakeDoor.setClosed();

        if(location == propLocation.Middle) { drive.followTrajectorySequence(dumpMPurplePixel);}
        else if(location == propLocation.Left) { drive.followTrajectorySequence(dumpLPurplePixel);}
        else { drive.followTrajectorySequence(dumpRPurplePixel); }

        if((location == propLocation.Right && !Blue) || (location == propLocation.Left && Blue))
            drive.followTrajectorySequence(extraForPurple);

        //robot.leftPixel.setDump();
        //lp.waitMillis(500);
        RobotLog.ii(RobotConstants.TAG_R, "left pixel isBlue" + Blue + "dump" + robot.pixel.LEFT_DUMP);

        if(Blue) {
            float lPixelPos = robot.pixel.pixel.getPosition();//153f;
            long dropperTime = System.currentTimeMillis();
            while (lPixelPos >= robot.pixel.LEFT_OUT && System.currentTimeMillis() - dropperTime < 1000) {
                RobotLog.ii(RobotConstants.TAG_R, "left pixel pos" + lPixelPos + "dump" + robot.pixel.LEFT_DUMP);
                RobotLog.ii(RobotConstants.TAG_R, "moving left pixel");
                lPixelPos -= 8;
                robot.pixel.setPos(lPixelPos);
                lp.waitMillis(3);
            }
        }
        else {
            float rPixelPos = robot.pixel.pixel.getPosition();//153f;
            long dropperTime = System.currentTimeMillis();
            while (rPixelPos <= robot.pixel.RIGHT_OUT && System.currentTimeMillis() - dropperTime < 500) {//hSlidesOut >= hSlides.MIN+10) {
                rPixelPos += 8;
                robot.pixel.setPos(rPixelPos);
                lp.waitMillis(3);
            }
        }

        robot.intake.slowIn();
        robot.intakeSmallTilt.setOut();
        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FOURTH);
        //robot.intakeSmallTilt.setPosition(robot.intakeSmallTilt.FIFTHP);
        if((location == propLocation.Right && !Blue) || (location == propLocation.Left && Blue))
            drive.followTrajectorySequence(bridgeGoToIntake);
        else
            drive.followTrajectorySequence(goToIntake);

        lp.waitMillis(175);


        robot.hslides.moveEncoderTo(hslides.START,-1);
        robot.intake.off();
        robot.intakeBigTilt.setTransfer();
        robot.intakeSmallTilt.setTransfer();
        robot.hslides.in();
        drive.followTrajectorySequence(dumpYellowPixel1);
        drive.followTrajectorySequence(turnCorrection1);
        robot.intakeSmallTilt.setOut();
        if(location == propLocation.Middle) { drive.followTrajectorySequence(dumpMYellowPixel2);}
        else if(location == propLocation.Left) { drive.followTrajectorySequence(dumpLYellowPixel2);}
        else { drive.followTrajectorySequence(dumpRYellowPixel2); }

        if(location == propLocation.Middle) { drive.followTrajectorySequence(turnCorrection2M);}
        else if(location == propLocation.Left) { drive.followTrajectorySequence(turnCorrection2L);}
        else { drive.followTrajectorySequence(turnCorrection2R); }

        /*robot.intakeDoor.setClosed();
        robot.depoDoor.setClosed();
        robot.intake.off();
        robot.vSlides.moveEncoderTo(DropHeight ? robot.vSlides.autoLevel : robot.vSlides.autoLevel+50, 1);*/
        //drive.followTrajectorySequence(dumpYellowPixel3);
        if(DropHeight){
            robot.vSlides.moveEncoderTo(robot.vSlides.autoLevel, 1);
        }else {
            robot.vSlides.moveEncoderTo(robot.vSlides.autoLevel + 150, 1);
        }
        lp.waitMillis(500);

        //robot.depo.setDepoOutVert();
        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
        robot.depo.setArmPos(robot.depo.ARM_OUT);
        if(location == propLocation.Middle) { drive.followTrajectorySequence(extraMPush);}
        else if(location == propLocation.Left) { drive.followTrajectorySequence(extraLPush);}
        else { drive.followTrajectorySequence(extraRPush); }

        //robot.depo.setArmPos(robot.depo.ARM_OUT);
        //robot.depo.setDepoOutVert();
        //lp.waitMillis(100);
        //robot.depo.setDepoOutFlat();
        //robot.depo.setWristPos(robot.depo.WRIST_FLAT);

        //robot.depo.setBothClawsOpen();
        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
        lp.waitMillis(250);

        robot.vSlides.moveEncoderTo(robot.vSlides.level3-200, 1);
        /*lp.waitMillis(100);

        //robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
        robot.depo.setDepoOutVert();
        lp.waitMillis(100);
        robot.depo.setDepoIn();
        //robot.depo.setArmPos(robot.depo.ARM_IN);
        lp.waitMillis(500);*/

//        robot.vSlides.moveEncoderTo(DropHeight ? robot.vSlides.autoLevel : robot.vSlides.autoLevel+50, 1);
        //lp.waitMillis(500);
        //robot.vSlides.moveEncoderTo(robot.vSlides.autoLevel, 1);
        //lp.waitMillis(600);

        //robot.depoTilt.setOut();
        //drive.followTrajectorySequence(dumpYellowPixel2);

//        robot.depoDoor.setOpen2();
//        lp.waitMillis(100);
//
//        robot.vSlides.moveEncoderTo(robot.vSlides.level4, 1);
        /*robot.vSlides.vSlidesF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.vSlides.vSlidesB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.vSlides.vSlidesB.setPower(1);
        robot.vSlides.vSlidesF.setPower(1);*/
        //lp.waitMillis(250);

        //drive.followTrajectorySequence(park);
//        robot.depoTilt.setIn();
        //slidesDown(lp);
        // robot.depo.setDepoOutVert();
        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
        robot.depo.setArmPos(robot.depo.ARM_OUT);
        lp.waitMillis(200);
        //robot.depo.setBothClawsClose();
        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
        if(ParkingLocation){ drive.followTrajectorySequence(wallPark); }
        else { drive.followTrajectorySequence(centerPark); }
        slidesDown(lp);
        //lowerSlidesThread(lp);
        robot.intakeBigTilt.setTransfer();
        robot.intakeSmallTilt.setTransfer();
        lp.waitMillis(200);
        robot.hang.setIn();

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