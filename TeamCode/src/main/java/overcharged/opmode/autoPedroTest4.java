package overcharged.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import overcharged.components.RobotMecanum;
import overcharged.components.propLocation;
import overcharged.config.RobotConstants;
import overcharged.drive.SampleMecanumDrive;
import overcharged.pedroPathing.util.SingleRunAction;
import overcharged.test.EasyOpenCVExample;
import overcharged.test.HSVPipeline;

import overcharged.pedroPathing.follower.Follower;
import overcharged.pedroPathing.localization.Pose;
import overcharged.pedroPathing.pathGeneration.BezierCurve;
import overcharged.pedroPathing.pathGeneration.BezierLine;
import overcharged.pedroPathing.pathGeneration.BezierPoint;
import overcharged.pedroPathing.pathGeneration.MathFunctions;
import overcharged.pedroPathing.pathGeneration.Path;
import overcharged.pedroPathing.pathGeneration.PathChain;
import overcharged.pedroPathing.pathGeneration.Point;
import overcharged.pedroPathing.util.Timer;


// Main Class
@Autonomous(name = "Pedro Auto Wall", group = "Autonomous")
public class autoPedroTest4 extends OpMode{

    private RobotMecanum robot;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry telems;
    SampleMecanumDrive drive;
    private Timer pathTimer, opmodeTimer, scanTimer, distanceSensorUpdateTimer, distanceSensorDecimationTimer;
    private SingleRunAction foldUp;

    //other stuff
    boolean mrBeast = false;


    //cam
    private int pathState;
    HSVPipeline detector;
    OpenCvWebcam webcam;
    EasyOpenCVExample.RingDeterminationPipeline pipeline;
    propLocation location = propLocation.Middle;

    //WaitLinear lp = new WaitLinear(this);

    /* LOCATION GUIDE
    • Field is a coordinate system spanning from the red human player wing (0,0) to the corner of the red backdrop (144,144)
    • center of the field is (72,72) therefore you can base values off of it(eg. 36 is a fourth of the field)
    • You can then use a ruler/measuring tape to find the exact values of specific poses you want. just add/substract it from the closest relative value(72, 36, 144) and keep it as preset poses
    */

    //All spikemark locations
    //
    //red left spike poses
    private Pose redLeftSideLeftSpikeMark = new Pose(36 + 72, -47.5 + 72);
    private Pose redLeftSideMiddleSpikeMark = new Pose(24.5 + 72, -36 + 72);
    private Pose redLeftSideRightSpikeMark = new Pose(36 + 72, -24.5 + 72);
    //
    //red right spike poses
    private Pose redRightSideLeftSpikeMark = new Pose(36 + 72, 0.5 + 72);
    private Pose redRightSideMiddleSpikeMark = new Pose(24.5 + 72, 12 + 72);
    private Pose redRightSideRightSpikeMark = new Pose(36 + 72, 23.5 + 72);
    //
    //blue left spike poses
    private Pose blueLeftSideLeftSpikeMark = new Pose(-36 + 72, 23.5 + 72);
    private Pose blueLeftSideMiddleSpikeMark = new Pose(-24.5 + 72, 12 + 72);
    private Pose blueLeftSideRightSpikeMark = new Pose(-36 + 72, 0.5 + 72);
    //
    //blue right spike poses
    private Pose blueRightSideLeftSpikeMark = new Pose(-36 + 72, -24.5 + 72);
    private Pose blueRightSideMiddleSpikeMark = new Pose(-24.5 + 72, -36 + 72);
    private Pose blueRightSideRightSpikeMark = new Pose(-36 + 72, -47.5 + 72);

    // backdrop april tag locations
    //
    //blue side
    private Pose blueLeftBackdrop = new Pose(-42.875 + 72, 60.75 + 72);
    private Pose blueMiddleBackdrop = new Pose(-36.75 + 72, 60.75 + 72);
    private Pose blueRightBackdrop = new Pose(-30.75 + 72, 60.75 + 72);
    //
    //red side
    private Pose redLeftBackdrop = new Pose(30.75 + 72, 60.75 + 72);
    private Pose redMiddleBackdrop = new Pose(36.75 + 72, 60.75 + 72);
    private Pose redRightBackdrop = new Pose(42.875 + 72, 60.75 + 72);

    // white pixel stack locations
    private Pose redOuterStack = new Pose(36 + 72, -72 + 72);
    private Pose redMiddleStack = new Pose(24 + 72, -72 + 72);
    private Pose redInnerStack = new Pose(12 + 72, -72 + 72);
    private Pose blueInnerStack = new Pose(-12 + 72, -72 + 72);
    private Pose blueMiddleStack = new Pose(-24 + 72, -72 + 72);
    private Pose blueOuterStack = new Pose(-36 + 72, -72 + 72);


    //Pose presets
    private Pose spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleBackdropGoalPose, secondCycleStackPose, secondCycleBackdropGoalPose;
    private Pose dropperPose, ready2Score;
    private Pose bDropBackAway;
    private Pose cycleOuterStack, cycleInnerStack;
    private Pose midTrussPose;
    private Pose initialspikeposeHeading = new Pose(0, 0, 180);

    private Pose startPose = new Pose(9.5, 72+16.75, Math.PI);

    private Point endPoint = new Point(60,56+72,Point.CARTESIAN);

    private Point moveOutPoint;
    private Point bridgePoint, bridgeReadyPoint;

    //Other presets
    private Follower follower;
    private Path scoreSpikeMark, initialScoreOnBackdrop, backAway, cycleOneMidTruss, cycleOneBridge, altCycleOneBridge, cycleOneBoard;
    private PathChain firstCycleToStack, firstCycleStackGrab, firstCycleScoreOnBackdrop, secondCycleToStack, secondCycleStackGrab, secondCycleScoreOnBackdrop;

    //Starting goal poses
    public void startingGoalPose() {
        switch (location) {
            case Left:
                spikeMarkGoalPose = new Pose(blueLeftSideLeftSpikeMark.getX() , blueLeftSideLeftSpikeMark.getY(), 0);
                moveOutPoint = new Point(blueLeftSideMiddleSpikeMark.getX()-20, blueLeftSideMiddleSpikeMark.getY(), Point.CARTESIAN);
                break;
            default:
            case Middle:
                spikeMarkGoalPose = new Pose(blueLeftSideMiddleSpikeMark.getX()-9 ,blueLeftSideMiddleSpikeMark.getY()+6, Math.toRadians(-90));//0);
                moveOutPoint = new Point(blueLeftSideMiddleSpikeMark.getX()-18, blueLeftSideMiddleSpikeMark.getY()+6, Point.CARTESIAN);
                break;
            case Right:
                spikeMarkGoalPose = new Pose(blueLeftSideRightSpikeMark.getX() , blueLeftSideRightSpikeMark.getY(), 0);
                moveOutPoint = new Point(blueLeftSideMiddleSpikeMark.getX()-20, blueLeftSideMiddleSpikeMark.getY(), Point.CARTESIAN);
                break;

        }
    }

    //backdrop poses
    public void backdropPoses() {
        switch (location) {
            case Left:
                initialBackdropGoalPose = new Pose(blueMiddleBackdrop.getX() - 0.75, blueMiddleBackdrop.getY() - 2.75,0);
                break;

            default:
            case Middle:
                initialBackdropGoalPose = new Pose(blueMiddleBackdrop.getX()-0, blueMiddleBackdrop.getY() - 26,Math.toRadians(-90));
                break;

            case Right:
                initialBackdropGoalPose = new Pose(blueMiddleBackdrop.getX() - 0.75, blueMiddleBackdrop.getY() - 2.75,0);
                break;

        }
        ready2Score = new Pose(blueMiddleBackdrop.getX(),blueMiddleBackdrop.getY()-30, Math.toRadians(-90));
    }


    // more presets
    //also init path sequence actions
    public void buildPaths() {
        switch (location) {
            case Left:

            default:
            case Middle:
                bDropBackAway = new Pose(blueMiddleBackdrop.getX()-0, blueMiddleBackdrop.getY()-28,0);
            case Right:

        }
        //poses and paths
        //TODO: clean this up


    }



    // Main pathing
    public void autoPath() {
        switch (pathState) {
            // Auto Body
            //
            case 10: // starts following the spike mark detected
                follower.followPath(scoreSpikeMark);
                hSlidesIn();
                setPathState(11);
                break;
            case 100: // empty test case
                telems.addLine("CASE 100 - IN TEST CASE!!");
                break;

            default:
                requestOpModeStop();
                break;
        }
        /*
        // makes robot park if auto about to end
        // TODO: Test
        if (opmodeTimer.getElapsedTimeSeconds() > 28) {
            foldUp.run();
        }
        */
    }


    // path setter
    public void setPathState(int state){
        pathState = state;
        pathTimer.resetTimer();
        autoPath();
    }

    // Distance Sensor Checker
    public void startDistanceSensorDisconnectDetection(int state) {
    }

    //loop de loop
    @Override
    public void loop() {
        follower.update();
        autoPath();
        telemetry.addLine("TValue: "+follower.getCurrentTValue());
        telemetry.addLine("Path: " + pathState);
    }

    // initialize robot
    @Override
    public void init() {

        // Timeout Safety Net
        foldUp = new SingleRunAction(()-> {
            if (Integer.parseInt(String.valueOf(pathState).substring(0,1)) < 4) setPathState(21);
        });

        // Robot things init
        telems = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);
        robot = new RobotMecanum(this, true, true);
        drive = new SampleMecanumDrive(hardwareMap);
        pathTimer = new Timer();

        // component init
        robot.hang.setIn();
        robot.depo.setFrontClawPos(robot.depo.FRONT_DUMP);
        robot.depo.setBackClawPos(robot.depo.BACK_DUMP);
        robot.pixel.setLeftIn();

        //Pose init
        startingGoalPose();
        backdropPoses();
        buildPaths();

        //cam init
        initCamera();
        this.detector = new HSVPipeline();
        webcam.setPipeline(detector);
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

        //follower init
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        //telem cam updates
        telemetry.addData("Prop Location", location);
        telemetry.update();
    }

    //loop de loop but initialized
    @Override
    public void init_loop() {

    }

    //start
    // opmode-auto  ---> auto_path / autobody
    // webcam
    // init robot start position, intake status
    // dump purple/yellow pixels
    // fetch pixels
    // dump pixels
    // park
    @Override
    public void start() {
        // starts auto paths
        setPathState(10);

        // just to make sure claw closed
        robot.depo.setFrontClawPos(robot.depo.FRONT_CLOSE);
        robot.depo.setBackClawPos(robot.depo.BACK_CLOSE);

        // safety net if auto doesn't start for some reason
        autoPath();
    }

    //camera init
    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new EasyOpenCVExample.RingDeterminationPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDevice();
        webcam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
    }

    //waiter
    public static void waitFor(int milliseconds) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < milliseconds) {
            // loop
        }
    }

    // Vslides reset function
    public void vSlidesDown(){
        robot.vSlides.vSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(!robot.vSlides.slideReachedBottom()){
            robot.vSlides.down();
        }
        robot.vSlides.forcestop();
        robot.vSlides.vSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Hslides reset function
    public void hSlidesIn(){
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

/*
TODO: :3
TODO: tune cases > 17
TODO: Get intake Working
TODO: Left and Right beacon paths
*/
