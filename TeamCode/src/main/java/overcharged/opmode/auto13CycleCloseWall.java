package overcharged.opmode;


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
import overcharged.drive.DriveConstants;
import overcharged.linear.util.SelectLinear;
import overcharged.linear.util.WaitLinear;
import overcharged.drive.SampleMecanumDrive;
import overcharged.test.EasyOpenCVExample;
import overcharged.test.HSVPipeline;
import overcharged.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="auto13CycleCloseWall")
public class auto13CycleCloseWall extends LinearOpMode {

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
    double waitTime = 0;
    boolean DropHeight = true;
    boolean ParkLocation = true;
    propLocation location = propLocation.Middle;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry telems;
    float xPurpleDump, yPurpleDump, xYellowDump, yYellowDump, xPark, yPark;
    TrajectorySequence test, dumpMPurplePixel, dumpLPurplePixel, dumpRPurplePixel, extraForPurple,
            dumpMYellowPixel, dumpLYellowPixel, dumpRYellowPixel, park, wallPark, centerPark, goUnder, goUnder2, goToDump, goToDump2, goToIntake, goToIntake2;
    Pose2d start = new Pose2d();

    //   @Override
    public void runOpMode() throws InterruptedException {
        try {
            telems = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);
            robot = new RobotMecanum(this, true, true);
            drive = new SampleMecanumDrive(hardwareMap);
            WaitLinear lp = new WaitLinear(this);
            initCamera(); // initializes camera and sets up pipeline for team shipping element detection
            robot.hang.setIn();
            robot.intakeDoor.setOpen();

            Blue = sl.selectAlliance();
            if(Blue){robot.pixel.setLeftIn();}
            else{robot.pixel.setRightIn();}
            DropHeight = sl.selectDropHeight(); //true = low, false = high drop
            waitTime = sl.adjustDelay();
            ParkLocation = sl.selctParkingLocation();

            float xWall = Blue? -4.5f:-3f;

            float yYellowDump = 90f;
            float xIntake = Blue ? -27.5f : -26;
            float yIntake = Blue? 16.5f : -16f;
            //MIDDLE
            float xMPurpleDump = Blue ? -29 : -29;
            float yMPurpleDump = Blue ? -2 : 1f;
            float xMYellowDump = Blue ? -26 : -26f;//-25: -22;

            //LEFT
            float xLPurpleDump = Blue ? -25 : -26;
            float yLPurpleDump = Blue ? -3.5f : -4.5f;//9.5f;
            float xLYellowDump = Blue ? -22f : -33f;//-18;

            //RIGHT
            float xRPurpleDump = Blue ? -26 : -25;
            float yRPurpleDump = Blue ? 4.5f : 0f;//-4;
            float xRYellowDump = Blue ? -33.5f : -18.9f;
            int dropHeight;


            if(DropHeight){
                dropHeight = robot.vSlides.autoLevel;
            }
            else{
                dropHeight = robot.vSlides.autoLevel + 150;
            }
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
            extraForPurple = drive.trajectorySequenceBuilder(Blue? dumpRPurplePixel.end() : dumpLPurplePixel.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(75, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineTo(new Vector2d(Blue? xRPurpleDump : xLPurpleDump, Blue? yRPurpleDump+8.5 : yLPurpleDump-9))
                    .build();
            dumpMYellowPixel = drive.trajectorySequenceBuilder(dumpMPurplePixel.end())
                    .lineToLinearHeading(new Pose2d(xMYellowDump, yYellowDump, Math.toRadians(Blue? 90 : -90)))
                    .addSpatialMarker(new Vector2d(xMYellowDump, Blue? yYellowDump+30:yYellowDump-30), () -> {
                        robot.vSlides.moveEncoderTo(dropHeight, 1);
                    })
                    .addSpatialMarker(new Vector2d(xMYellowDump, Blue? yYellowDump+18:yYellowDump-18), () -> {
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_OUT);
                    })
                    .build();
            dumpLYellowPixel = drive.trajectorySequenceBuilder(dumpLPurplePixel.end())
                    .lineToLinearHeading(new Pose2d(xLYellowDump, yYellowDump, Math.toRadians(Blue? 90 : -90)))
                    .addSpatialMarker(new Vector2d(xMYellowDump, Blue? yYellowDump+30:yYellowDump-30), () -> {
                        robot.vSlides.moveEncoderTo(dropHeight, 1);
                    })
                    .addSpatialMarker(new Vector2d(xMYellowDump, Blue? yYellowDump+18:yYellowDump-18), () -> {
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_OUT);
                    })
                    .build();
            dumpRYellowPixel = drive.trajectorySequenceBuilder(dumpRPurplePixel.end())
                    .lineToLinearHeading(new Pose2d(xRYellowDump, yYellowDump, Math.toRadians(Blue? 90 : -90)))
                    .addSpatialMarker(new Vector2d(xMYellowDump, Blue? yYellowDump+30:yYellowDump-30), () -> {
                        robot.vSlides.moveEncoderTo(dropHeight, 1);
                    })
                    .addSpatialMarker(new Vector2d(xMYellowDump, Blue? yYellowDump+18:yYellowDump-18), () -> {
                        //robot.depo.setDepoOutVert();
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_OUT);
                    })
                    .build();
            goUnder = drive.trajectorySequenceBuilder(dumpRYellowPixel.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(300, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    //.lineTo(new Vector2d(Blue?xWall:xWall, Blue?-(yYellowDump-3):yYellowDump-3))
                    //.lineToLinearHeading(new Pose2d(xWall, Blue? -(yYellowDump-10):yYellowDump-10, Math.toRadians(Blue ? 90 : -90)))
                    .splineToConstantHeading(new Vector2d(xWall,  Blue? -(yYellowDump-10):yYellowDump-10), Math.toRadians(Blue ? 90 : -90),
                            SampleMecanumDrive.getVelocityConstraint(180, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .addSpatialMarker(new Vector2d(xWall, Blue? -(yYellowDump - 5): yYellowDump - 5), () -> {
                        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_IN);
                    })
                    .addSpatialMarker(new Vector2d(xWall, Blue? -(yYellowDump - 20): yYellowDump - 17), () -> {
                        // robot.depo.setBothClawsOpen();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        vSlidesDown(lp);
                    })
                    .lineTo(new Vector2d(xWall, Blue? yIntake-18:yMPurpleDump))


                    .build();
            goToIntake = drive.trajectorySequenceBuilder(goUnder.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(300, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .splineToConstantHeading(new Vector2d(Blue? xIntake-2: xIntake-2, Blue ? yIntake-8 : yIntake + 8), Math.toRadians(Blue ? 90 : -90),
                            SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    // .lineToLinearHeading(new Pose2d(Blue? xIntake: xIntake-2, Blue?yIntake-8:yIntake+8, Math.toRadians(Blue? 90 : -90)))
                    .lineTo(new Vector2d(Blue? xIntake-2:xIntake-2, yIntake))
                    .addSpatialMarker(new Vector2d(xIntake, Blue? (yIntake-10):(yIntake+10)), () -> {
                        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.THIRD);
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        robot.intakeDoor.setClosed();
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue ? yIntake-8 : yIntake+8), () -> {
                        robot.intake.slowIn();
                        robot.intakeSmallTilt.setOut();
                        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FOURTH+1);
                        robot.intakeDoor.setClosed();
                    })
                    .build();
            goToDump = drive.trajectorySequenceBuilder(new Pose2d(goToIntake.end().getX(), goToIntake.end().getY(), Math.toRadians(Blue? 90:-90)))
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(300, Math.PI * 2, DriveConstants.TRACK_WIDTH))

                    .lineTo(new Vector2d(xWall, Blue? yIntake-5:yIntake+5))
                    .lineTo(new Vector2d(xWall, Blue? -(yYellowDump-8):yYellowDump-8))
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-2:(yIntake+2)), () -> {
                        robot.intake.in();
                        robot.intakeDoor.setClosed();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        robot.intakeBigTilt.setTransfer();
                        robot.intakeSmallTilt.setTransfer();
                        hSlidesIn(lp);
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-6:(yIntake+6)), () -> {
                        // robot.depo.setBothClawsOpen();
                        robot.intakeDoor.setOpen();
                        robot.intake.out();
                        hSlidesIn(lp);
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-7:(yIntake+7)), () -> {
                        robot.intake.off();
                        robot.intakeDoor.setOpen();
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-20:(yIntake+20)), () -> {
                        robot.intake.slowIn();
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? -(yYellowDump-30):(yYellowDump-30)), () -> {
                        //robot.intakeDoor.setClosed();
                        // robot.depoDoor.setClosed
                        ///  robot.depo.setBothClawsClose();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                        robot.intake.off();
                        //robot.vSlides.moveEncoderTo(robot.vSlides.level1, 1);
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? -(yYellowDump-25):(yYellowDump-25)), () -> {
                        robot.vSlides.moveEncoderTo(robot.vSlides.level1+200, 1);
                    })
                    .lineToLinearHeading(new Pose2d(Blue?xLYellowDump-4:xRYellowDump-2, Blue? -(yYellowDump-2) : yYellowDump-1, Math.toRadians(Blue? 90:-90)))//xMYellowDump, Blue? (yYellowDump+13) : yYellowDump-13), Math.toRadians(Blue? 90:-90))
                    .addSpatialMarker(new Vector2d(Blue?xLYellowDump+1:xRYellowDump+1, Blue? -(yYellowDump-18) : yYellowDump-18), () -> {
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_OUT);
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? -(yYellowDump-15) : yYellowDump-15), () -> {
                        // robot.depo.setAngularLeft();
                        robot.depo.setWristPos(robot.depo.WRIST_FLAT);
                    })
                    .build();
            goUnder2 = drive.trajectorySequenceBuilder(goToDump.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(300, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    //.lineTo(new Vector2d(Blue?xWall:xWall, Blue?-(yYellowDump-3):yYellowDump-3))
                    //.lineToLinearHeading(new Pose2d(xWall, Blue? -(yYellowDump-10):yYellowDump-10, Math.toRadians(Blue ? 90 : -90)))
                    .splineToConstantHeading(new Vector2d(xWall,  Blue? -(yYellowDump-10):yYellowDump-10), Math.toRadians(Blue ? 90 : -90),
                            SampleMecanumDrive.getVelocityConstraint(180, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .addSpatialMarker(new Vector2d(xWall, Blue? -(yYellowDump - 5): yYellowDump - 5), () -> {
                        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_IN);
                    })
                    .addSpatialMarker(new Vector2d(xWall, Blue? -(yYellowDump - 20): yYellowDump - 17), () -> {
                        // robot.depo.setBothClawsOpen();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        vSlidesDown(lp);
                    })
                    .lineTo(new Vector2d(xWall, Blue? yIntake-18:yMPurpleDump))


                    .build();
            goToIntake2 = drive.trajectorySequenceBuilder(goUnder.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(300, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .splineToConstantHeading(new Vector2d(Blue? xIntake-2: xIntake-2, Blue ? yIntake-8 : yIntake + 8), Math.toRadians(Blue ? 90 : -90),
                            SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    // .lineToLinearHeading(new Pose2d(Blue? xIntake: xIntake-2, Blue?yIntake-8:yIntake+8, Math.toRadians(Blue? 90 : -90)))
                    .lineTo(new Vector2d(Blue? xIntake-2:xIntake-2, yIntake))
                    .addSpatialMarker(new Vector2d(xIntake, Blue? (yIntake-10):(yIntake+10)), () -> {
                        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.THIRD);
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        robot.intakeDoor.setClosed();
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue ? yIntake-8 : yIntake+8), () -> {
                        robot.intake.slowIn();
                        robot.intakeSmallTilt.setOut();
                        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.SECOND);
                        robot.intakeDoor.setClosed();
                    })
                    .build();
            goToDump2 = drive.trajectorySequenceBuilder(new Pose2d(goToIntake2.end().getX(), goToIntake2.end().getY(), Math.toRadians(Blue? 90:-90)))
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(300, Math.PI * 2, DriveConstants.TRACK_WIDTH))

                    .lineTo(new Vector2d(xWall, Blue? yIntake-5:yIntake+5))
                    .lineTo(new Vector2d(xWall, Blue? -(yYellowDump-8):yYellowDump-8))
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-2:(yIntake+2)), () -> {
                        robot.intake.in();
                        robot.intakeDoor.setClosed();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        robot.intakeBigTilt.setTransfer();
                        robot.intakeSmallTilt.setTransfer();
                        hSlidesIn(lp);
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-6:(yIntake+6)), () -> {
                        // robot.depo.setBothClawsOpen();
                        robot.intakeDoor.setOpen();
                        robot.intake.out();
                        hSlidesIn(lp);
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-7:(yIntake+7)), () -> {
                        robot.intake.off();
                        robot.intakeDoor.setOpen();
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-20:(yIntake+20)), () -> {
                        robot.intake.slowIn();
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? -(yYellowDump-30):(yYellowDump-30)), () -> {
                        //robot.intakeDoor.setClosed();
                        // robot.depoDoor.setClosed
                        ///  robot.depo.setBothClawsClose();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                        robot.intake.off();
                        //robot.vSlides.moveEncoderTo(robot.vSlides.level1, 1);
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? -(yYellowDump-25):(yYellowDump-25)), () -> {
                        robot.vSlides.moveEncoderTo(robot.vSlides.level1+200, 1);
                    })
                    .lineToLinearHeading(new Pose2d(Blue?xLYellowDump-4:xRYellowDump-2, Blue? -(yYellowDump-2) : yYellowDump-1, Math.toRadians(Blue? 90:-90)))//xMYellowDump, Blue? (yYellowDump+13) : yYellowDump-13), Math.toRadians(Blue? 90:-90))
                    .addSpatialMarker(new Vector2d(Blue?xLYellowDump+1:xRYellowDump+1, Blue? -(yYellowDump-18) : yYellowDump-18), () -> {
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_OUT);
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? -(yYellowDump-15) : yYellowDump-15), () -> {
                        // robot.depo.setAngularLeft();
                        robot.depo.setWristPos(robot.depo.WRIST_FLAT);
                    })
                    .build();
            wallPark = drive.trajectorySequenceBuilder(new Pose2d(Blue? goToDump2.end().getX() : goToDump2.end().getX(), Blue ? goToDump2.end().getY()+9 : goToDump2.end().getY()-9, Math.toRadians(Blue? 90 : -90)))
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(75, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToConstantHeading(new Vector2d(2, Blue? yYellowDump+7:yYellowDump-7))
                    .addSpatialMarker(new Vector2d(3, Blue? yYellowDump+3:yYellowDump), () -> {
                        //robot.depo.setArmPos(robot.depo.ARM_IN);
                        //robot.depo.setDepoIn();
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_IN);
                    })
                    .build();
            centerPark = drive.trajectorySequenceBuilder(new Pose2d(goToDump2.end().getX(), Blue ? goToDump2.end().getY()+9 : goToDump2.end().getY()-9, Math.toRadians(Blue? 90 : -90)))
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(105, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToConstantHeading(new Vector2d(-48, Blue? yYellowDump+7:yYellowDump-7))
                    .addSpatialMarker(new Vector2d(-25, Blue? yYellowDump:yYellowDump), () -> {
                        //robot.depo.setDepoIn();
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_IN);
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
                location = detector.getLocation(!Blue, false);
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
                    location = detector.getLocation(!Blue, false);
                    currentTime = System.currentTimeMillis();
                }

                robot.hang.setIn();

                robot.vSlides.reset(robot.vSlides.vSlides);

                //detector.reset();
                telemetry.addData("Blue alliance", Blue);
                telemetry.addData("Prop location", location);
                telemetry.update();

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
        lp.waitMillis(waitTime);

//        robot.depo.setBothClawsClose();
        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);

        if(location == propLocation.Middle) { drive.followTrajectorySequence(dumpMPurplePixel);}
        else if(location == propLocation.Left) { drive.followTrajectorySequence(dumpLPurplePixel);}
        else { drive.followTrajectorySequence(dumpRPurplePixel); }

        if((location == propLocation.Right && Blue) || (location == propLocation.Left && !Blue))
            drive.followTrajectorySequence(extraForPurple);

        RobotLog.ii(RobotConstants.TAG_R, "left pixel isBlue" + Blue + "dump" + robot.pixel.LEFT_DUMP);

        if(Blue) {
            float lPixelPos = robot.pixel.pixel.getPosition();//153f;
            long dropperTime = System.currentTimeMillis();
            while (lPixelPos >= robot.pixel.LEFT_OUT && System.currentTimeMillis() - dropperTime < 1000) {
                RobotLog.ii(RobotConstants.TAG_R, "left pixel pos" + lPixelPos + "dump" + robot.pixel.LEFT_DUMP);
                RobotLog.ii(RobotConstants.TAG_R, "moving left pixel");
                lPixelPos -= 3;
                robot.pixel.setPos(lPixelPos);
                lp.waitMillis(5);
            }
        }
        else {
            float rPixelPos = robot.pixel.pixel.getPosition();//153f;
            long dropperTime = System.currentTimeMillis();
            while (rPixelPos <= robot.pixel.RIGHT_OUT && System.currentTimeMillis() - dropperTime < 500) {//hSlidesOut >= hSlides.MIN+10) {
                rPixelPos += 3;
                robot.pixel.setPos(rPixelPos);
                lp.waitMillis(5);
            }
        }
/*
        if(DropHeight){
            robot.vSlides.moveEncoderTo(robot.vSlides.autoLevel, 1);
        } else {
            robot.vSlides.moveEncoderTo(robot.vSlides.autoLevel+150, 1);
        }
        lp.waitMillis(50);
*/
        //robot.depo.setDepoOutVert();
        //robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
        //  robot.depo.setArmPos(robot.depo.ARM_OUT);
        if(location == propLocation.Middle) { drive.followTrajectorySequence(dumpMYellowPixel);}
        else if(location == propLocation.Left) { drive.followTrajectorySequence(dumpLYellowPixel);}
        else { drive.followTrajectorySequence(dumpRYellowPixel); }
        //robot.depo.setBothClawsOpen();
        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
        lp.waitMillis(50);
        // Cycle 1
        drive.followTrajectorySequence(goUnder);
        drive.followTrajectorySequence(goToIntake);
        robot.intakeDoor.setClosed();
        lp.waitMillis(50);
        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.SECOND+6);
        lp.waitMillis(50);
        drive.followTrajectorySequence(goToDump);
        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
        lp.waitMillis(200);

        // Cycle 2
        drive.followTrajectorySequence(goUnder2);
        drive.followTrajectorySequence(goToIntake2);
        robot.intakeDoor.setClosed();
        lp.waitMillis(50);
        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FIFTH+12);
        lp.waitMillis(50);
        drive.followTrajectorySequence(goToDump2);
        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
        lp.waitMillis(200);

        //drive.followTrajectorySequence(park);
        if(ParkLocation){
            drive.followTrajectorySequence(wallPark);
        } else {
            drive.followTrajectorySequence(centerPark);
        }
        lp.waitMillis(100);

        robot.vSlides.vSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        long slideDownTime = System.currentTimeMillis();
        RobotLog.ii(RobotConstants.TAG_R, "reached bottom? " + robot.vSlides.switchSlideDown.isTouch() + " time elapsed " + (System.currentTimeMillis() - slideDownTime));
        while(!robot.vSlides.switchSlideDown.isTouch() && System.currentTimeMillis() - slideDownTime < 2000){
            RobotLog.ii(RobotConstants.TAG_R, "reached bottom? " + robot.vSlides.switchSlideDown.isTouch() + " power " + robot.vSlides.vSlides.getPower() + " time elapsed " + (System.currentTimeMillis() - slideDownTime));
            robot.vSlides.down();
        }
        robot.vSlides.setPower(0);
        robot.vSlides.forcestop();
        robot.vSlides.reset(robot.vSlides.vSlides);


        robot.hang.setIn();
        lp.waitMillis(30000-System.currentTimeMillis()+startTime);
    }

    public void lowerSlidesThread(WaitLinear lp) { // asynchronously start raising the slides
        Runnable lowerSlidesThread = new vSlidesThread(0.6f, false, lp, this, robot);
        Thread thread = new Thread(lowerSlidesThread);
        thread.start();
    }
    public void vSlidesDown(WaitLinear lp){
        robot.vSlides.vSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(!robot.vSlides.slideReachedBottom()){
            robot.vSlides.down();
        }
        robot.vSlides.forcestop();
        robot.vSlides.vSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void hSlidesIn(WaitLinear lp){
        robot.hslides.hslidesR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hslides.hslidesL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(!robot.hslides.switchSlideDown.isTouch()){
            robot.hslides.inAuto();
        }

        robot.hslides.hslidesR.setPower(0);
        robot.hslides.hslidesL.setPower(0);

        robot.hslides.hslidesR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hslides.hslidesL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}