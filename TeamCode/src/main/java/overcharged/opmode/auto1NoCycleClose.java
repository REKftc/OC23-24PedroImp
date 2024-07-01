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
@Autonomous(name="auto1NoCycleClose")
public class auto1NoCycleClose extends LinearOpMode {

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
            dumpMYellowPixel, dumpLYellowPixel, dumpRYellowPixel, park, wallPark, centerPark;
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

            float yYellowDump = Blue? -38f:38;//91f;
            float xIntake = Blue ? -27.5f : -26;

            //MIDDLE
            float xMPurpleDump = Blue? -29: -29;
            float yMPurpleDump = Blue? -3: 3f;
            float xMYellowDump = Blue? -25: -25;

            //LEFT
            float xLPurpleDump = Blue? -26: -26;
            float yLPurpleDump = Blue? -10.5f: -3;//9.5f;
            float xLYellowDump = Blue? -18f: -33f;//-18;

            //RIGHT
            float xRPurpleDump = Blue? -26: -26;
            float yRPurpleDump = Blue? 4f: 10.5f;//-4;
            float xRYellowDump = Blue? -33.5f: -17f;//-36;
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
            wallPark = drive.trajectorySequenceBuilder(new Pose2d(Blue? dumpLYellowPixel.end().getX() : dumpRYellowPixel.end().getX(), Blue ? dumpLYellowPixel.end().getY()+9 : dumpRYellowPixel.end().getY()-9, Math.toRadians(Blue? 90 : -90)))
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(75, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToConstantHeading(new Vector2d(2, Blue? yYellowDump+7:yYellowDump-7))
                    .addSpatialMarker(new Vector2d(3, Blue? yYellowDump+3:yYellowDump), () -> {
                        //robot.depo.setArmPos(robot.depo.ARM_IN);
                        //robot.depo.setDepoIn();
                        robot.depo.setWristPos(robot.depo.WRIST_IN_VERT);
                        robot.depo.setArmPos(robot.depo.ARM_IN);
                    })
                    .build();
            centerPark = drive.trajectorySequenceBuilder(new Pose2d(dumpMYellowPixel.end().getX(), Blue ? dumpMYellowPixel.end().getY()+9 : dumpMYellowPixel.end().getY()-9, Math.toRadians(Blue? 90 : -90)))
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
}