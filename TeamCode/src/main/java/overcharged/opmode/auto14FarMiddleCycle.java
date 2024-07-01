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
@Autonomous(name="auto14FarMiddleCycle")
public class auto14FarMiddleCycle extends LinearOpMode {

    private RobotMecanum robot;
    SampleMecanumDrive drive;
    SelectLinear sl = new SelectLinear(this);
    int detectionWaitTime = 450;
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
    boolean cycleNum = true;
    propLocation location = propLocation.Middle;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry telems;
    TrajectorySequence dumpMPurplePixel, dumpLPurplePixel, dumpRPurplePixel, extraForPurple, goToIntake,
            bridgeGoToIntake, dumpYellowPixel1, dumpMYellowPixel2, dumpLYellowPixel2, dumpRYellowPixel2,
            extraMPush, extraLPush, extraRPush, cycleIntake1, cycleIntake2, additionalCycleDump, extraPush2,
            turnCorrection1, turnCorrection2M, turnCorrection2L, turnCorrection2R,turnCorrection3M,
            turnCorrection3L, turnCorrection3R, goUnder, goToIntake2, goToDump, wallPark, centerPark;
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
            if (Blue) {
                robot.pixel.setLeftIn();
            } else {
                robot.pixel.setRightIn();
            }
            DropHeight = sl.selectDropHeight(); //true = low, false = high drop
            ParkingLocation = sl.selctParkingLocation(); //true = wall, false = center
            cycleNum = sl.selectCycleNumber();
            waitTime = sl.adjustDelay();
            //notWithRV = sl.selectRV();

            float xWall = Blue? -4.5f:-4f;
            float newXIntake = Blue ? -51:-51;
            int hSlideLength = 500;//Blue? 1440 : 1440;//1460;

            float yYellowDump = 88f;
            float xIntake = Blue ? -26f : -26;
            float yIntake = Blue? 16f : -16.5f;
            //MIDDLE
            float xMPurpleDump = Blue ? -29 : -29;
            float yMPurpleDump = Blue ? -2 : 1f;
            float xMYellowDump = Blue ? -27 : -26f;//-25: -22;

            //LEFT
            float xLPurpleDump = Blue ? -25 : -26;
            float yLPurpleDump = Blue ? -3.5f : -4.5f;//9.5f;
            float xLYellowDump = Blue ? -16.5f : -32.5f;//-18;

            //RIGHT
            float xRPurpleDump = Blue ? -26 : -25;
            float yRPurpleDump = Blue ? 4.5f : 0f;//-4;
            float xRYellowDump = Blue ? -30f : -17f;//-28.5f: -19.5f;//-36;

            //initialize trajectories
            dumpMPurplePixel = drive.trajectorySequenceBuilder(start)
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(150, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(xMPurpleDump, yMPurpleDump, Math.toRadians(Blue ? 90 : -90)))
                    .build();
            dumpLPurplePixel = drive.trajectorySequenceBuilder(start)
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(150, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(xLPurpleDump, yLPurpleDump, Math.toRadians(Blue ? 90 : -90)))
                    .build();
            dumpRPurplePixel = drive.trajectorySequenceBuilder(start)
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(150, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(xRPurpleDump, yRPurpleDump, Math.toRadians(Blue ? 90 : -90)))
                    .build();
            extraForPurple = drive.trajectorySequenceBuilder(Blue ? dumpLPurplePixel.end() : dumpRPurplePixel.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(150, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineTo(new Vector2d(Blue ? xLPurpleDump : xRPurpleDump, Blue ? yLPurpleDump - 14.5 : yRPurpleDump + 19))
                    .build();
            goToIntake = drive.trajectorySequenceBuilder(Blue ? dumpRPurplePixel.end() : dumpLPurplePixel.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(300, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(xIntake, yIntake+1, Math.toRadians(Blue ? 90 : -90)))
                    .addSpatialMarker(new Vector2d(xIntake, Blue ? yIntake-8 : yIntake+8), () -> {
                        //robot.intake.slowIn();
                        robot.intakeSmallTilt.setOut();
                        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FOURTH+3);
                    })
                    .build();
            bridgeGoToIntake = drive.trajectorySequenceBuilder(extraForPurple.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(300, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(xIntake, yIntake+1, Math.toRadians(Blue ? 90 : -90)))
                    .addSpatialMarker(new Vector2d(xIntake, Blue ? yIntake-10 : yIntake+10), () -> {
                        //robot.intake.slowIn();
                        robot.intakeSmallTilt.setOut();
                        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FOURTH+3);
                    })
                    .build();
            telemetry.addLine("1");
            telemetry.update();

            dumpYellowPixel1 = drive.trajectorySequenceBuilder(goToIntake.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(200, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(newXIntake, Blue ? 17 : -17, Math.toRadians(Blue? 93.5:-92.5)))
                    //.lineToConstantHeading(new Vector2d(newXIntake, Blue ? 17 : -17))
                    .addSpatialMarker(new Vector2d(Blue ? xIntake + 4 : xIntake + 4, Blue ? 10 : -10), () -> {
                        //robot.intake.in();
                        robot.intakeBigTilt.setTransfer();
                        robot.intakeSmallTilt.setTransfer();
                    })
                    .addSpatialMarker(new Vector2d(Blue ? xIntake + 4 : xIntake + 4, Blue ? 10 : -10), () -> {

                        robot.intake.out();
                        //robot.intakeDoor.setOpen();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        //robot.intake.in();
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        hSlidesIn(lp);
                        //robot.intakeSmallTilt.setPosition(intakeSmallTilt.DUMP_EXTRA);
                        //robot.intakeBigTilt.setPosition(intakeBigTilt.DUMP_EXTRA);
                    })
                    .addSpatialMarker(new Vector2d(Blue ? xIntake + 4.4 : xIntake + 4.4, Blue ? 10 : -10), () -> {
                        robot.intake.off();

                        //robot.intakeSmallTilt.setPosition(intakeSmallTilt.DUMP_EXTRA);
                        //robot.intakeBigTilt.setPosition(intakeBigTilt.DUMP_EXTRA);
                    })
                    .addSpatialMarker(new Vector2d(Blue ? xIntake + 10 : xIntake + 10, Blue ? 10 : -10), () -> {
                        //robot.intake.in();

                        //robot.intakeSmallTilt.setPosition(intakeSmallTilt.DUMP_EXTRA);
                        //robot.intakeBigTilt.setPosition(intakeBigTilt.DUMP_EXTRA);
                    })
                    .build();
            /*turnCorrection1 = drive.trajectorySequenceBuilder(new Pose2d(dumpYellowPixel1.end().getX() + 1, Blue ? dumpYellowPixel1.end().getY() + 1 : dumpYellowPixel1.end().getY() - 1, Math.toRadians(Blue ? 90 : -90)))
                    .lineToLinearHeading(new Pose2d(Blue ? -3.5 : -3, Blue ? 17 : -17, Math.toRadians(Blue ? 90 : -90)))
                    .build();*/
            telemetry.addLine("2");
            telemetry.update();


            dumpMYellowPixel2 = drive.trajectorySequenceBuilder(dumpYellowPixel1.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(200, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineTo(new Vector2d(newXIntake, Blue ? -63 : 63),
                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    //.splineTo(new Vector2d(Blue? xIntake-21 :xIntake-21,Blue? -63:63), Math.toRadians(Blue? 90 : -90))
                    .addSpatialMarker(new Vector2d(newXIntake, Blue ? 3 : -3), () -> {
                        //robot.vSlides.moveEncoderTo(100, 1);
                        //robot.hslides.moveEncoderTo(hslides.START,1);
                        hSlidesIn(lp);
                        robot.intakeDoor.setOpen();
                        //robot.depoDoor.setOpen2();
                        //robot.depo.setBothClawsOpen();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        //robot.intake.in();
                    })
                    .addSpatialMarker(new Vector2d(newXIntake, Blue ? -62 : 62), () -> {
                        //robot.intakeDoor.setClosed();
                        //robot.depoDoor.setClosed();
                        //robot.depo.setBothClawsClose();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                        robot.intake.off();
                        //robot.vSlides.moveEncoderTo(DropHeight ? robot.vSlides.autoLevel : robot.vSlides.autoLevel+50, 1);
                    })
                    .splineToConstantHeading(new Vector2d(xMYellowDump+4, Blue ? -(yYellowDump - 13) : yYellowDump - 13), Math.toRadians(Blue ? 90 : -90),
                            SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    /*.addSpatialMarker(new Vector2d(xMYellowDump, Blue? -(yYellowDump-14) : yYellowDump-14), () -> {
                        robot.vSlides.moveEncoderTo(robot.vSlides.autoLevel+100, 1);
                    })*/
                    .build();
            dumpLYellowPixel2 = drive.trajectorySequenceBuilder(dumpYellowPixel1.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(200, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineTo(new Vector2d(newXIntake, Blue ? -62 : 63),
                            SampleMecanumDrive.getVelocityConstraint(180, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    //.splineTo(new Vector2d(Blue? xIntake-21 :xIntake-21,Blue? -63:63), Math.toRadians(Blue? 90 : -90))
                    .addSpatialMarker(new Vector2d(newXIntake, Blue ? 5 : -5), () -> {
                        //robot.vSlides.moveEncoderTo(100, 1);
                        //robot.hslides.moveEncoderTo(hslides.START,1);
                        robot.intakeDoor.setOpen();
                        hSlidesIn(lp);

                        //robot.depoDoor.setOpen2();
                        //robot.depo.setBothClawsOpen();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        //robot.intake.in();
                    })
                    .addSpatialMarker(new Vector2d(newXIntake, Blue ? -61 : 62), () -> {
                        //robot.intakeDoor.setClosed();
                        //robot.depoDoor.setClosed();
                        //robot.BothClawsClose();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                        robot.intake.off();
                        //robot.vSlides.moveEncoderTo(DropHeight ? robot.vSlides.autoLevel : robot.vSlides.autoLevel+50, 1);
                    })
                    .splineToConstantHeading(new Vector2d(xLYellowDump+4.2, Blue ? -(yYellowDump - 17) : yYellowDump - 13), Math.toRadians(Blue ? 90 : -90),
                            SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    /*.addSpatialMarker(new Vector2d(xLYellowDump, Blue? -(yYellowDump-14) : yYellowDump-14), () -> {
                        robot.vSlides.moveEncoderTo(robot.vSlides.autoLevel+100, 1);
                    })*/
                    .build();
            dumpRYellowPixel2 = drive.trajectorySequenceBuilder(dumpYellowPixel1.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(200, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineTo(new Vector2d(newXIntake, Blue ? -63 : 63),
                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    //.splineTo(new Vector2d(Blue? xIntake-21 :xIntake-21,Blue? -63:63), Math.toRadians(Blue? 90 : -90))
                    .addSpatialMarker(new Vector2d(newXIntake, Blue ? 5 : -5), () -> {
                        //robot.vSlides.moveEncoderTo(100, 1);
                        //robot.hslides.moveEncoderTo(hslides.START,1);
                        hSlidesIn(lp);
                        robot.intakeDoor.setOpen();
                        //robot.depoDoor.setOpen2();
                        //robot.depo.setBothClawsOpen();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        //robot.intake.in();
                    })
                    .addSpatialMarker(new Vector2d(newXIntake, Blue ? -62 : 62), () -> {
                        //robot.intakeDoor.setClosed();
                        //robot.depoDoor.setClosed();
                        //robot.depo.setBothClawsClose();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                        robot.intake.off();
                        //robot.vSlides.moveEncoderTo(DropHeight ? robot.vSlides.autoLevel : robot.vSlides.autoLevel+50, 1);
                    })
                    .splineToConstantHeading(new Vector2d(xRYellowDump-2, Blue ? -(yYellowDump - 13) : yYellowDump - 13), Math.toRadians(Blue ? 90 : -90),
                            SampleMecanumDrive.getVelocityConstraint(180, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    /*.addSpatialMarker(new Vector2d(xRYellowDump, Blue? -(yYellowDump-14) : yYellowDump-14), () -> {
                        robot.vSlides.moveEncoderTo(robot.vSlides.autoLevel+100, 1);
                    })*/
                    .build();
//            turnCorrection2M = drive.trajectorySequenceBuilder(new Pose2d(dumpMYellowPixel2.end().getX() + 1, Blue ? dumpMYellowPixel2.end().getY() + 1 : dumpMYellowPixel2.end().getY() - 1, Math.toRadians(Blue ? 90 : -90)))
//                    .lineToLinearHeading(new Pose2d(xMYellowDump, Blue ? -(yYellowDump - 13) : yYellowDump - 13, Math.toRadians(Blue ? 90 : -90)))
//                    .build();
//            turnCorrection2L = drive.trajectorySequenceBuilder(new Pose2d(dumpLYellowPixel2.end().getX() + 1, Blue ? dumpLYellowPixel2.end().getY() + 1 : dumpLYellowPixel2.end().getY() - 1, Math.toRadians(Blue ? 90 : -90)))
//                    .lineToLinearHeading(new Pose2d(xLYellowDump, Blue ? -(yYellowDump - 13) : yYellowDump - 13, Math.toRadians(Blue ? 90 : -90)))
//                    .build();
//            turnCorrection2R = drive.trajectorySequenceBuilder(new Pose2d(dumpRYellowPixel2.end().getX() + 1, Blue ? dumpRYellowPixel2.end().getY() + 1 : dumpRYellowPixel2.end().getY() - 1, Math.toRadians(Blue ? 90 : -90)))
//                    .lineToLinearHeading(new Pose2d(xRYellowDump, Blue ? -(yYellowDump - 13) : yYellowDump - 13, Math.toRadians(Blue ? 90 : -90)))
//                    .build();
            telemetry.addLine("3");
            telemetry.update();

            extraMPush = drive.trajectorySequenceBuilder(dumpMYellowPixel2.end())
                    .lineToLinearHeading(new Pose2d(xMYellowDump, Blue ? -yYellowDump : yYellowDump, Math.toRadians(Blue ? 90 : -90)))
                    /*.addSpatialMarker(new Vector2d(xMYellowDump, Blue? -(yYellowDump-7) : yYellowDump-7), () -> {
                        //robot.depo.setArmPos(robot.depo.ARM_OUT);
                        robot.depo.setDepoOutVert();
                    })*/
                    .addSpatialMarker(new Vector2d(xMYellowDump, Blue ? -(yYellowDump - 8) : yYellowDump - 8), () -> {
                        //robot.depo.setWristPos(robot.depo.WRIST_FLAT);
                        //robot.depo.setDepoOutOppVert();
                        robot.depo.setArmPos(robot.depo.ARM_OUT);
                        robot.depo.setWristPos(robot.depo.WRIST_OPP_VERT);

                    })
                    .build();
            extraLPush = drive.trajectorySequenceBuilder(dumpLYellowPixel2.end())
                    .lineToLinearHeading(new Pose2d(xLYellowDump, Blue ? -yYellowDump+1.3 : yYellowDump, Math.toRadians(Blue ? 90 : -90)))
                    /*.addSpatialMarker(new Vector2d(xMYellowDump, Blue? -(yYellowDump-7) : yYellowDump-7), () -> {
                        //robot.depo.setArmPos(robot.depo.ARM_OUT);
                        robot.depo.setDepoOutVert();
                    })*/
                    .addSpatialMarker(new Vector2d(xMYellowDump-2, Blue ? -(yYellowDump - 8) : yYellowDump - 8), () -> {
                        //robot.depo.setWristPos(robot.depo.WRIST_FLAT);
                        //robot.depo.setDepoOutOppVert();
                        robot.depo.setArmPos(robot.depo.ARM_OUT);
                        robot.depo.setWristPos(robot.depo.WRIST_OPP_VERT);
                    })
                    .build();
            extraRPush = drive.trajectorySequenceBuilder(dumpRYellowPixel2.end())
                    .lineToLinearHeading(new Pose2d(xRYellowDump, Blue ? -yYellowDump : yYellowDump, Math.toRadians(Blue ? 90 : -90)))
                    /*.addSpatialMarker(new Vector2d(xMYellowDump, Blue? -(yYellowDump-7) : yYellowDump-7), () -> {
                        //robot.depo.setArmPos(robot.depo.ARM_OUT);
                        robot.depo.setDepoOutVert();
                    })*/
                    .addSpatialMarker(new Vector2d(xMYellowDump, Blue ? -(yYellowDump - 8) : yYellowDump - 8), () -> {
                        //robot.depo.setWristPos(robot.depo.WRIST_FLAT);
                        //robot.depo.setDepoOutOppVert();
                        robot.depo.setArmPos(robot.depo.ARM_OUT);
                        robot.depo.setWristPos(robot.depo.WRIST_OPP_VERT);
                    })
                    .build();
            telemetry.addLine("4");
            telemetry.update();
            newXIntake = -48;
            goUnder = drive.trajectorySequenceBuilder(Blue? new Pose2d(extraRPush.end().getX(), extraRPush.end().getY()+8, Math.toRadians(90)) : new Pose2d(extraLPush.end().getX(), extraLPush.end().getY()-8, Math.toRadians(-90)))//!Blue?extraLPush.end():extraRPush.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(300, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    //.lineTo(new Vector2d(Blue?xWall:xWall, Blue?-(yYellowDump-3):yYellowDump-3))
                    //.lineToLinearHeading(new Pose2d(xWall, Blue? -(yYellowDump-10):yYellowDump-10, Math.toRadians(Blue ? 90 : -90)))
                    .splineToConstantHeading(new Vector2d(newXIntake,  Blue? -(yYellowDump-10):yYellowDump-10), Math.toRadians(Blue ? 92 : -90),
                            SampleMecanumDrive.getVelocityConstraint(180, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .addSpatialMarker(new Vector2d(newXIntake, Blue? -(yYellowDump - 5): yYellowDump - 5), () -> {
                        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_IN);
                    })
                    .addSpatialMarker(new Vector2d(newXIntake, Blue? -(yYellowDump - 20): yYellowDump - 17), () -> {
                        // robot.depo.setBothClawsOpen();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        vSlidesDown(lp);
                        robot.hslides.moveEncoderTo(hSlideLength, .95f);
                    })
                    .addSpatialMarker(new Vector2d(newXIntake, Blue? yIntake-30:yIntake+30), () -> {
                        //hSlidesIn(lp);
                    })
                    .lineTo(new Vector2d(newXIntake-2, Blue? -6:6))//yIntake+1:yIntake-1))
                    /*.addSpatialMarker(new Vector2d(newXIntake, Blue? (yIntake-15):(yIntake+15)), () -> {
                        //robot.hslides.in();
                        hSlidesIn(lp);
                        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FOURTH+1);
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        robot.intakeDoor.setClosed();
                    })*/
                    .addSpatialMarker(new Vector2d(newXIntake-2, Blue ? -11 : 11), () -> {
                        robot.intakeSmallTilt.setOut();
                        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FIFTH);
                        robot.intakeDoor.setClosed();
                    })
                    .addSpatialMarker(new Vector2d(newXIntake-2, Blue ? -8 : 8), () -> {
                        //hSlidesIn(lp);
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        robot.intakeDoor.setClosed();
                        //robot.intake.slowIn();
                        /*robot.intakeSmallTilt.setOut();
                        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FIFTH);
                        robot.intakeDoor.setClosed();*/
                    })

                    .build();
            telemetry.addLine("5");
            telemetry.update();

            /*goToIntake2 = drive.trajectorySequenceBuilder(goUnder.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(300, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .splineTo(new Vector2d(Blue? xIntake+11: xIntake+5, Blue ? yIntake-2 : yIntake), Math.toRadians(Blue ? 127 : -120),
                            SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    // .lineToLinearHeading(new Pose2d(Blue? xIntake: xIntake-2, Blue?yIntake-8:yIntake+8, Math.toRadians(Blue? 90 : -90)))
                    //.lineTo(new Vector2d(Blue? xIntake-2:xIntake-3, Blue?yIntake-2:yIntake))
                    .addSpatialMarker(new Vector2d(xIntake, Blue? (yIntake-15):(yIntake+15)), () -> {
                        //robot.hslides.in();
                        hSlidesIn(lp);
                        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FOURTH+1);
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        robot.intakeDoor.setClosed();
                    })
                    .addSpatialMarker(new Vector2d(xIntake, Blue ? yIntake-8 : yIntake+8), () -> {
                        robot.intake.slowIn();
                        robot.intakeSmallTilt.setOut();
                        robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FOURTH-3);
                        robot.intakeDoor.setClosed();
                    })
                    .build();*/
            goToDump = drive.trajectorySequenceBuilder(goUnder.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(300, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    /*     .splineTo(new Vector2d(xWall, Blue ? yIntake-20 : yIntake+20), Math.toRadians(Blue ? 90 : -90),
                                 SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),

                                 SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                         )*/
                    //.lineToLinearHeading(new Pose2d(newXIntake, Blue?yIntake-20 : yIntake+20,Math.toRadians(Blue ? 90 : -90)))
                    .lineToLinearHeading(new Pose2d(newXIntake, Blue? -(yYellowDump-12):yYellowDump-12,Math.toRadians(Blue ? 90 : -90)))
                    /*.addSpatialMarker(new Vector2d(newXIntake, Blue? yIntake-2:(yIntake)), () -> {
                        hSlidesIn(lp);
                        robot.intake.in();
                        robot.intakeDoor.setClosed();
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        //robot.intakeBigTilt.setTransfer();
                        //robot.intakeSmallTilt.setTransfer();
                        hSlidesIn(lp);
                    })
                    .addSpatialMarker(new Vector2d(newXIntake, Blue? yIntake-4:(yIntake+4)), () -> {
                        hSlidesIn(lp);
                    })
                    .addSpatialMarker(new Vector2d(newXIntake, Blue? yIntake-6:(yIntake+6)), () -> {
                        // robot.depo.setBothClawsOpen()
                        // robot.intakeDoor.setOpen();;
                        hSlidesIn(lp);
                        robot.intake.out();
                    })
                    .addSpatialMarker(new Vector2d(newXIntake, Blue? yIntake-7:(yIntake+7)), () -> {
                        hSlidesIn(lp);
                        robot.intake.off();
                        // robot.intakeDoor.setOpen();
                    })
                    .addSpatialMarker(new Vector2d(newXIntake, Blue? yIntake-20:(yIntake+20)), () -> {
                        hSlidesIn(lp);
                        robot.intake.in();
                        robot.intakeDoor.setOpen();
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);

                    })
                    .addSpatialMarker(new Vector2d(newXIntake, Blue? yIntake-25:(yIntake+25)), () -> {
                        hSlidesIn(lp);
                    })*/
                    .addSpatialMarker(new Vector2d(newXIntake, Blue? -(yYellowDump-30):(yYellowDump-30)), () -> {
                        //robot.intakeDoor.setClosed();
                        // robot.depoDoor.setClosed
                        ///  robot.depo.setBothClawsClose();
                        hSlidesIn(lp);
                        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                        //robot.intake.in();

                        //robot.vSlides.moveEncoderTo(robot.vSlides.level1, 1);
                    })

                    /* .splineToConstantHeading(new Vector2d(xWall-0.7,  Blue? -(yYellowDump-10):yYellowDump-10), Math.toRadians(Blue ? 90 : -90),
                             SampleMecanumDrive.getVelocityConstraint(180, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                             SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                     )*/
                    .addSpatialMarker(new Vector2d(newXIntake, Blue? -(yYellowDump-14):(yYellowDump-14)), () -> {
                        hSlidesIn(lp);
                        robot.vSlides.moveEncoderTo(robot.vSlides.level1+200, 1);
                        robot.intake.off();
                    })
//                    .lineToLinearHeading(new Pose2d(Blue?xLYellowDump-6:xRYellowDump-2, Blue? -(yYellowDump-4) : yYellowDump-1, Math.toRadians(Blue? 90:-90)))//xMYellowDump, Blue? (yYellowDump+13) : yYellowDump-13), Math.toRadians(Blue? 90:-90))
                    .lineToLinearHeading(new Pose2d(newXIntake+6, Blue? -(yYellowDump+3.5) : yYellowDump+3.5, Math.toRadians(Blue? 148:-145)))//xMYellowDump, Blue? (yYellowDump+13) : yYellowDump-13), Math.toRadians(Blue? 90:-90))

                    .addSpatialMarker(new Vector2d(newXIntake, Blue? -(yYellowDump-18) : yYellowDump-18), () -> {
                        hSlidesIn(lp);
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_OUT);
                    })
                    .addSpatialMarker(new Vector2d(newXIntake, Blue? -(yYellowDump-6) : yYellowDump-6), () -> {
                        hSlidesIn(lp);
                        robot.depo.setAngularPos(!Blue?robot.depo.ANGULAR_RIGHT:robot.depo.ANGULAR_LEFT);
                    })
                    .build();
            telemetry.addLine("6");
            telemetry.update();

            wallPark = drive.trajectorySequenceBuilder(!Blue? new Pose2d(goToDump.end().getX(), goToDump.end().getY()+6, Math.toRadians(90)) : new Pose2d(goToDump.end().getX(), goToDump.end().getY()-5, Math.toRadians(-90)))
                    //.lineTo(new Vector2d(Blue?xLYellowDump-6:xRYellowDump-2, Blue? -(yYellowDump-5) : yYellowDump-6))
                    .addSpatialMarker(new Vector2d(!Blue?xLYellowDump-6:xRYellowDump-2, Blue? -(yYellowDump-4)+6 : yYellowDump-15-2), () -> {
                        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                    })
                    .lineToConstantHeading(new Vector2d(Blue?xWall+2:xWall, Blue? -(yYellowDump-4) : yYellowDump-5))

                    .addSpatialMarker(new Vector2d(-25, Blue? -(yYellowDump-4):yYellowDump-5), () -> {
                        //robot.depo.setDepoIn();
                        robot.depo.setAngularPos(robot.depo.ANGULAR_STRAIGHT);
                        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
                        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_IN);
                    })
                    .addSpatialMarker(new Vector2d(-18, Blue? -(yYellowDump-4):yYellowDump-5), () -> {
                        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
                        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
                        vSlidesDown(lp);
                        hSlidesIn(lp);
                    })
                    .build();
            centerPark = drive.trajectorySequenceBuilder(!Blue? new Pose2d(goToDump.end().getX(), goToDump.end().getY()+5, Math.toRadians(90)) : new Pose2d(goToDump.end().getX(), goToDump.end().getY()-5, Math.toRadians(-90)))
                    .lineToConstantHeading(new Vector2d(newXIntake-3, Blue? -(yYellowDump):yYellowDump))
                    .addSpatialMarker(new Vector2d(-25, Blue? yYellowDump:yYellowDump), () -> {
                        // robot.depo.setDepoIn();
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_IN);
                    })
                    .addSpatialMarker(new Vector2d(-40, Blue? yYellowDump+2:yYellowDump-2), () -> {
                        vSlidesDown(lp);
                        hSlidesIn(lp);
                    })
                    .build();
            telemetry.addLine("7");
            telemetry.update();


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


        //robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FOURTH);
        //robot.intakeSmallTilt.setPosition(robot.intakeSmallTilt.FIFTHP-5);
        if((location == propLocation.Right && !Blue) || (location == propLocation.Left && Blue))
            drive.followTrajectorySequence(bridgeGoToIntake);
        else
            drive.followTrajectorySequence(goToIntake);

        lp.waitMillis(20);


//        robot.hslides.moveEncoderTo(hslides.START,-1);
        //robot.intake.off();
        //   robot.intakeBigTilt.setTransfer();
        // robot.intakeSmallTilt.setTransfer();

        //robot.hslides.in();
        drive.followTrajectorySequence(dumpYellowPixel1);
        //drive.followTrajectorySequence(turnCorrection1);
        //robot.intakeSmallTilt.setOut();
        if(location == propLocation.Middle) { drive.followTrajectorySequence(dumpMYellowPixel2);}
        else if(location == propLocation.Left) { drive.followTrajectorySequence(dumpLYellowPixel2);}
        else { drive.followTrajectorySequence(dumpRYellowPixel2); }

        lp.waitMillis(200);

//        if(location == propLocation.Middle) { drive.followTrajectorySequence(turnCorrection2M);}
//        else if(location == propLocation.Left) { drive.followTrajectorySequence(turnCorrection2L);}
//        else { drive.followTrajectorySequence(turnCorrection2R); }

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
        //lp.waitMillis(100);




        if(location == propLocation.Middle) { drive.followTrajectorySequence(extraMPush);}
        else if(location == propLocation.Left) { drive.followTrajectorySequence(extraLPush);}
        else { drive.followTrajectorySequence(extraRPush); }
        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
        lp.waitMillis(600);
        if(cycleNum) {
            //robot.vSlides.moveEncoderTo(robot.vSlides.level3-100, 1);
            drive.followTrajectorySequence(goUnder);
            lp.waitMillis(40); // 70
            robot.intakeBigTilt.setPosition(robot.intakeBigTilt.FOURTH);
            lp.waitMillis(350); //150

            hSlidesIn(lp);
            //robot.intake.in();
            robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
            robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
            robot.intakeBigTilt.setTransfer();
            robot.intakeSmallTilt.setTransfer();
            lp.waitMillis(1000);
            robot.intakeDoor.setOpen();
            drive.followTrajectorySequence(goToDump);

            robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
            robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
            robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
            robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
            lp.waitMillis(200);
        }

        robot.intakeBigTilt.setTransfer();
        robot.intakeSmallTilt.setTransfer();

        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);
        robot.depo.setDepoIn();

        lp.waitMillis(400);

        vSlidesDown(lp);

        robot.hang.setIn();

        lp.waitMillis(30000-System.currentTimeMillis()+startTime);
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

