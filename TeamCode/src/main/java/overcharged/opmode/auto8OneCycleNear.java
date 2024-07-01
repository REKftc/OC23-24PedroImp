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
@Autonomous(name="auto8OneCycleNear")
public class auto8OneCycleNear extends LinearOpMode {

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
    boolean ParkingLocation = true;
    propLocation location = propLocation.Middle;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry telems;
    float xPurpleDump, yPurpleDump, xYellowDump, yYellowDump, xPark, yPark;
    TrajectorySequence test, dumpMPurplePixel, dumpLPurplePixel, dumpRPurplePixel, extraForPurple, goToIntakeWall,
            dumpMYellowPixel, dumpLYellowPixel, dumpRYellowPixel, wallPark, goToIntake,
            goToIntakeM, goToIntakeL, goToIntakeR, goToDump1, extraPush2, centerPark, goToIntake2, goToDump2;
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
            robot.intakeDoor.setOpen();
            // robot.depo.setBothClawsClose();
            robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
            robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);

            Blue = sl.selectAlliance();
            if(Blue){robot.pixel.setLeftIn();}
            else{robot.pixel.setRightIn();}
            ParkingLocation = sl.selctParkingLocation();
            DropHeight = sl.selectDropHeight(); //true = low, false = high drop
            waitTime = sl.adjustDelay();

            float yYellowDump = Blue? -38f:38;//91f;
            float xIntake = Blue ? -53.25f:-51f;//-53.25f;
            float yMidIntake = 18f;
            float yIntake = Blue? 50f : -50f;
            int hSlideLength = 475;//Blue? 1440 : 1440;//1460;

            //MIDDLE
            float xMPurpleDump = Blue? -29: -29;
            float yMPurpleDump = Blue? -3: 3f;
            float xMYellowDump = Blue? -24: -25.5f;//-26

            //LEFT
            float xLPurpleDump = Blue? -26: -26;
            float yLPurpleDump = Blue? -10.5f: -3;//9.5f;
            float xLYellowDump = Blue? -17f: -32f;//-18;

            //RIGHT
            float xRPurpleDump = Blue? -26: -26;
            float yRPurpleDump = Blue? 4f: 10.5f;//-4;
            float xRYellowDump = Blue? -33.5f: -17f;//-36;


            //initialize trajectories
            dumpMPurplePixel = drive.trajectorySequenceBuilder(start)
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(90, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(xMPurpleDump, yMPurpleDump, Math.toRadians(Blue? 90 : -90)))
                    .build();
            dumpLPurplePixel = drive.trajectorySequenceBuilder(start)
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(90, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(xLPurpleDump, yLPurpleDump, Math.toRadians(Blue? 90 : -90)))
                    .build();
            dumpRPurplePixel = drive.trajectorySequenceBuilder(start)
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(90, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(xRPurpleDump, yRPurpleDump, Math.toRadians(Blue? 90 : -90)))
                    .build();
            extraForPurple = drive.trajectorySequenceBuilder(Blue? dumpRPurplePixel.end() : dumpLPurplePixel.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(90, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineTo(new Vector2d(Blue? xRPurpleDump : xLPurpleDump, Blue? yRPurpleDump+8.5 : yLPurpleDump-9))
                    .build();
            dumpMYellowPixel = drive.trajectorySequenceBuilder(dumpMPurplePixel.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(85, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(xMYellowDump, yYellowDump, Math.toRadians(Blue? 90 : -90)))
                    .addSpatialMarker(new Vector2d(xMYellowDump, Blue? yYellowDump+18:yYellowDump-18), () -> {
                        //robot.depo.setDepoOutVert();
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_OUT);
                    })
                    .build();
            dumpLYellowPixel = drive.trajectorySequenceBuilder(dumpLPurplePixel.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(85, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(xLYellowDump, yYellowDump, Math.toRadians(Blue? 90 : -90)))
                    .addSpatialMarker(new Vector2d(xMYellowDump, Blue? yYellowDump+18:yYellowDump-18), () -> {
                        //robot.depo.setDepoOutVert();
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_OUT);
                    })
                    .build();
            dumpRYellowPixel = drive.trajectorySequenceBuilder(dumpRPurplePixel.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(85, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(xRYellowDump, yYellowDump, Math.toRadians(Blue? 90 : -90)))
                    .addSpatialMarker(new Vector2d(xMYellowDump, Blue? yYellowDump+18:yYellowDump-18), () -> {
                        //robot.depo.setDepoOutVert();
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_OUT);
                    })
                    .build();
            goToIntakeM = drive.trajectorySequenceBuilder((new Pose2d(dumpMYellowPixel.end().getX(), dumpMYellowPixel.end().getY(), Math.toRadians(Blue?92:-92))))
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(105, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineTo(new Vector2d(dumpMYellowPixel.end().getX(), Blue?dumpMYellowPixel.end().getY()+5:dumpMYellowPixel.end().getY()-5))
                    .splineToConstantHeading(new Vector2d(xIntake, Blue? -yMidIntake:yMidIntake), Math.toRadians(Blue? 92.5 : -92.5))
                    .addSpatialMarker(new Vector2d(xIntake, Blue? -(yMidIntake+5):(yMidIntake+5)), () -> {
                        //robot.depo.setDepoIn();
                        //robot.depo.setBothClawsClose();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_IN);
                        //robot.depo.setArmPos(robot.depo.ARM_IN);
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? -(yMidIntake+1):(yMidIntake+1)), () -> {
                        //robot.depo.setBothClawsOpen();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        slidesDown(lp);
                        robot.hslides.moveEncoderTo(hSlideLength, .85f);
                    })
                    .lineTo(new Vector2d(xIntake, yIntake))
                    .build();
            goToIntakeL = drive.trajectorySequenceBuilder((new Pose2d(dumpLYellowPixel.end().getX(), dumpLYellowPixel.end().getY(), Math.toRadians(Blue?91:-92))))
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(105, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineTo(new Vector2d(dumpLYellowPixel.end().getX(), Blue?dumpLYellowPixel.end().getY()+3:dumpLYellowPixel.end().getY()-5))
                    .splineToConstantHeading(new Vector2d(xIntake, Blue? -(yMidIntake+2):yMidIntake), Math.toRadians(Blue? 92 : -92.5))
                    .addSpatialMarker(new Vector2d(xIntake, Blue? -(yMidIntake+5):(yMidIntake+5)), () -> {
                        //robot.depo.setDepoIn();
                        //robot.depo.setArmPos(robot.depo.ARM_IN);
                       // robot.depo.setBothClawsClose();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_IN);
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? -(yMidIntake+1):(yMidIntake+1)), () -> {
                       // robot.depo.setBothClawsOpen();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        slidesDown(lp);
                        robot.hslides.moveEncoderTo(hSlideLength, .85f);
                    })
                    .lineTo(new Vector2d(xIntake, yIntake))
                    .build();
            goToIntakeR = drive.trajectorySequenceBuilder((new Pose2d(dumpRYellowPixel.end().getX(), dumpRYellowPixel.end().getY(), Math.toRadians(Blue?92.5:-92))))
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(105, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineTo(new Vector2d(dumpRYellowPixel.end().getX(), Blue?dumpRYellowPixel.end().getY()+5:dumpLYellowPixel.end().getY()-3))
                    .splineToConstantHeading(new Vector2d(xIntake, Blue? -yMidIntake:(yMidIntake+2)), Math.toRadians(Blue? 94 : -94))
                    .addSpatialMarker(new Vector2d(xIntake, Blue? -(yMidIntake+5):(yMidIntake+5)), () -> {
                        //robot.depo.setDepoIn();
                       // robot.depo.setBothClawsClose();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_IN);
                        //robot.depo.setArmPos(robot.depo.ARM_IN);
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? -(yMidIntake+1):(yMidIntake+1)), () -> {
                        //robot.depo.setBothClawsOpen();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        slidesDown(lp);
                        robot.hslides.moveEncoderTo(hSlideLength, .85f);
                    })
                    .lineTo(new Vector2d(xIntake, yIntake))
                    .build();

            /*goToIntake = drive.trajectorySequenceBuilder(Blue? (new Pose2d(dumpRYellowPixel.end().getX(), dumpRYellowPixel.end().getY()+5, Math.toRadians(92.5))) : new Pose2d(dumpLYellowPixel.end().getX(), dumpLYellowPixel.end().getY()-3, Math.toRadians(-92.5)))
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(75, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .splineToConstantHeading(new Vector2d(xIntake, Blue? -yMidIntake:yMidIntake), Math.toRadians(Blue? 92.5 : -92.5))
                    .addSpatialMarker(new Vector2d(xIntake, Blue? -(yMidIntake+5):(yMidIntake+5)), () -> {
                        robot.depo.setDepoIn();
                        //robot.depo.setArmPos(robot.depo.ARM_IN);
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? -(yMidIntake+1):(yMidIntake+1)), () -> {
                        slidesDown(lp);
                        robot.hslides.moveEncoderTo(1460, 1);
                    })
                    .lineTo(new Vector2d(xIntake, yIntake))
                    .build();*/
            goToDump1 = drive.trajectorySequenceBuilder(goToIntakeM.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(105, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineTo(new Vector2d(xIntake, Blue? -yMidIntake:yMidIntake-1))
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-2:(yIntake+2)), () -> {
                        robot.intakeDoor.setClosed();
                        robot.intakeBigTilt.setTransfer();
                        robot.intakeSmallTilt.setTransfer();
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-6:(yIntake+6)), () -> {
                        //robot.depo.setBothClawsOpen();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        robot.intake.out();
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-7:(yIntake+7)), () -> {
                        robot.intake.off();
                    })
                    /*.addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-10:(yIntake+10)), () -> {
                        robot.intake.slowIn();
                    })*/
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-10:(yIntake+10)), () -> {
                        robot.intake.slowIn();
                        robot.intakeDoor.setOpen();
                        // robot.depoDoor.setOpen2();
                        //robot.depo.setBothClawsOpen();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-40:(yIntake+40)), () -> {
                        //robot.intakeDoor.setClosed();
                        // robot.depoDoor.setClosed();
                        //robot.depo.setBothClawsClose();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                        robot.intake.off();
                        //robot.vSlides.moveEncoderTo(robot.vSlides.level1, 1);
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? -(yMidIntake-1):yMidIntake-2), () -> {
                        robot.vSlides.moveEncoderTo(robot.vSlides.level1+100, 1);
                    })
                    .splineToConstantHeading(new Vector2d(xMYellowDump, Blue? (yYellowDump+13) : yYellowDump-13), Math.toRadians(Blue? 90:-90))
                    .build();
            extraPush2 = drive.trajectorySequenceBuilder(goToDump1.end())
                    .lineToLinearHeading(new Pose2d(xMYellowDump-2, yYellowDump, Math.toRadians(Blue? 90 : -90)))
                    /*.addSpatialMarker(new Vector2d(xMYellowDump, Blue? (yYellowDump+11) : yYellowDump-11), () -> {
                        robot.depo.setWristPos(robot.depo.WRIST_FLAT);
                    })*/
                    .build();
            goToIntake2 = drive.trajectorySequenceBuilder(new Pose2d(extraPush2.end().getX(), extraPush2.end().getY(), Math.toRadians(Blue? 92:-90.5)))
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(105, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineTo(new Vector2d(xMYellowDump, Blue?yYellowDump+5:yYellowDump-5))
                    .splineToConstantHeading(new Vector2d(Blue? xIntake+1: xIntake, Blue? -yMidIntake:yMidIntake), Math.toRadians(Blue? 92 : -95))
                    .addSpatialMarker(new Vector2d(xIntake, Blue? -(yMidIntake+5):(yMidIntake+5)), () -> {
                        robot.intakeDoor.setClosed();
                        //robot.depo.setDepoIn();
                       // robot.depo.setBothClawsClose();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_IN);
                        //robot.depo.setArmPos(robot.depo.ARM_IN);
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? -(yMidIntake+1):(yMidIntake+1)), () -> {
                        //robot.depo.setBothClawsOpen();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        slidesDown(lp);
                        robot.hslides.moveEncoderTo(hSlideLength, .85f);
                    })
                    .lineTo(new Vector2d(Blue? xIntake+1:xIntake, yIntake))
                    .addSpatialMarker(new Vector2d(xIntake, Blue? (yIntake-1):(yIntake+1)), () -> {
                        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.THIRD);
                    })
                    .build();
            goToDump2 = drive.trajectorySequenceBuilder(new Pose2d(goToIntake2.end().getX(), goToIntake2.end().getY(), Math.toRadians(Blue? 90:-90)))
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(105, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineTo(new Vector2d(xIntake, Blue? -yMidIntake:yMidIntake-1))
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-2:(yIntake+2)), () -> {
                        robot.intakeDoor.setClosed();
                        robot.intakeBigTilt.setTransfer();
                        robot.intakeSmallTilt.setTransfer();
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-6:(yIntake+6)), () -> {
                        // robot.depo.setBothClawsOpen();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        robot.intake.out();
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-7:(yIntake+7)), () -> {
                        robot.intake.off();
                    })
                    /*.addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-10:(yIntake+10)), () -> {
                        robot.intake.slowIn();
                    })*/
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-10:(yIntake+10)), () -> {
                        robot.intake.slowIn();
                        robot.intakeDoor.setOpen();
                        // robot.depoDoor.setOpen2();
                        //robot.depo.setBothClawsOpen();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? yIntake-40:(yIntake+40)), () -> {
                        //robot.intakeDoor.setClosed();
                        // robot.depoDoor.setClosed();
                        ///  robot.depo.setBothClawsClose();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                        robot.intake.off();
                        //robot.vSlides.moveEncoderTo(robot.vSlides.level1, 1);
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue? -(yMidIntake-1):yMidIntake-2), () -> {
                        robot.vSlides.moveEncoderTo(robot.vSlides.level1+200, 1);
                    })
                    //.splineToConstantHeading(new Vector2d(Blue? xMYellowDump-2: xMYellowDump-2, Blue? (yYellowDump+13) : yYellowDump-13), Math.toRadians(Blue? 90:-90))
                    .lineToLinearHeading(new Pose2d(Blue?xIntake+5:xIntake+12, Blue? (yYellowDump-5) : yYellowDump+3, Math.toRadians(Blue? 148:-145)))

                    .build();
            wallPark = drive.trajectorySequenceBuilder(new Pose2d(dumpMYellowPixel.end().getX(), Blue ? dumpMYellowPixel.end().getY()+7 : dumpMYellowPixel.end().getY()-7, Math.toRadians(Blue?148:-145)))
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(105, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToConstantHeading(new Vector2d(0, Blue? yYellowDump+2:yYellowDump-2))
                    .addSpatialMarker(new Vector2d(-15, Blue? yYellowDump:yYellowDump), () -> {
                        //robot.depo.setDepoIn();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_IN);
                    })
                    .addSpatialMarker(new Vector2d(-8, Blue? yYellowDump+2:yYellowDump-2), () -> {
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        slidesDown(lp);
                    })
                    .build();
            centerPark = drive.trajectorySequenceBuilder(new Pose2d(dumpMYellowPixel.end().getX(), Blue ? dumpMYellowPixel.end().getY()+7 : dumpMYellowPixel.end().getY()-7, Math.toRadians(Blue? 90 : -90)))
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(105, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToConstantHeading(new Vector2d(-42, Blue? yYellowDump+2:yYellowDump-2))
                    .addSpatialMarker(new Vector2d(-25, Blue? yYellowDump:yYellowDump), () -> {
                        //robot.depo.setDepoIn();
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_IN);
                    })
                    .addSpatialMarker(new Vector2d(-39, Blue? yYellowDump+2:yYellowDump-2), () -> {
                        //robot.depo.setBothClawsOpen();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
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

        if(location == propLocation.Middle) { drive.followTrajectorySequence(dumpMPurplePixel);}
        else if(location == propLocation.Left) { drive.followTrajectorySequence(dumpLPurplePixel);}
        else { drive.followTrajectorySequence(dumpRPurplePixel); }

        if((location == propLocation.Right && Blue) || (location == propLocation.Left && !Blue))
            drive.followTrajectorySequence(extraForPurple);
        //robot.pixel.setRightOut();
        //robot.pixel.setLeftOut();
        //robot.leftPixel.setDump();
        //lp.waitMillis(500);
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

        robot.vSlides.moveEncoderTo(robot.vSlides.autoLevel-40, 1);
        //lp.waitMillis(400);

        //lp.waitMillis(1000);
        //robot.depo.setArmPos(robot.depo.ARM_OUT);
        if(location == propLocation.Middle) { drive.followTrajectorySequence(dumpMYellowPixel);}
        else if(location == propLocation.Left) { drive.followTrajectorySequence(dumpLYellowPixel);}
        else { drive.followTrajectorySequence(dumpRYellowPixel); }
        //robot.depoDoor.setOpen2();
        //robot.depo.setBothClawsOpen();
        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
        lp.waitMillis(100);

        //cycle starts
        robot.intakeDoor.setClosed();
        robot.intakeSmallTilt.setOut();
        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FIFTH);
        robot.intake.slowIn();
        if(location == propLocation.Middle) { drive.followTrajectorySequence(goToIntakeM);}
        else if(location == propLocation.Left) { drive.followTrajectorySequence(goToIntakeL);}
        else { drive.followTrajectorySequence(goToIntakeR); }
        lp.waitMillis(70);
        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FOURTH);
        lp.waitMillis(150);

        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FIFTH+8);
        robot.hslides.moveEncoderTo(hslides.START,-1);
        //lp.waitMillis(5000);
        /*lp.waitMillis(150);
        robot.intakeDoor.setClosed();
        robot.intakeBigTilt.setTransfer();
        robot.intakeSmallTilt.setTransfer();*/
        drive.followTrajectorySequence(goToDump1);
        //robot.depo.setDepoOutVert();
        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
        robot.depo.setArmPos(robot.depo.ARM_OUT);
        drive.followTrajectorySequence(extraPush2);
        //robot.depo.setBothClawsOpen();
        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
        lp.waitMillis(150);
        //cycle endssd

        //cycle starts
        robot.intakeDoor.setClosed();
        robot.intakeSmallTilt.setOut();
        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FIFTH+20);
        robot.intake.slowIn();
        //robot.depo.setArmPos(robot.depo.ARM_IN);
        drive.followTrajectorySequence(goToIntake2);
        //robot.intakeBigTilt.setPosition(robot.intakeBigTilt.THIRD);
        //lp.waitMillis(100);
        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.SECOND);
        lp.waitMillis(250);

        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FIFTH+8);
        robot.hslides.moveEncoderTo(hslides.START,-1);
        /*lp.waitMillis(150);
        robot.intakeDoor.setClosed();
        robot.intakeBigTilt.setTransfer();
        robot.intakeSmallTilt.setTransfer();*/
        drive.followTrajectorySequence(goToDump2);
        //robot.depo.setDepoOutVert();
        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
        robot.depo.setArmPos(robot.depo.ARM_OUT);
        drive.followTrajectorySequence(extraPush2);
        //robot.depo.setBothClawsOpen();
        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
        lp.waitMillis(100);
        //cycle ends

        robot.vSlides.moveEncoderTo(robot.vSlides.level3-200, 1);
        if(ParkingLocation){ drive.followTrajectorySequence(wallPark); }
        else { drive.followTrajectorySequence(centerPark); }
        /*lp.waitMillis(250);
        robot.depo.setDepoIn();
        lp.waitMillis(250);
        slidesDown(lp);*/

        robot.hang.setIn();
        //lowerSlidesThread(lp);
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
}