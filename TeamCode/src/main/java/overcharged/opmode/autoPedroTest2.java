package overcharged.opmode;

import static overcharged.config.RobotConstants.ROBOT_FRONT_LENGTH;
import static overcharged.config.RobotConstants.ROBOT_BACK_LENGTH;
import static overcharged.config.RobotConstants.TAG_SL;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

import overcharged.pedroPathing.follower.Follower;
import overcharged.pedroPathing.localization.Pose;
import overcharged.pedroPathing.pathGeneration.BezierCurve;
import overcharged.pedroPathing.pathGeneration.BezierLine;
import overcharged.pedroPathing.pathGeneration.BezierPoint;
import overcharged.pedroPathing.pathGeneration.MathFunctions;
import overcharged.pedroPathing.pathGeneration.Path;
import overcharged.pedroPathing.pathGeneration.PathChain;
import overcharged.pedroPathing.pathGeneration.Point;
import overcharged.pedroPathing.util.SingleRunAction;
import overcharged.pedroPathing.util.Timer;
import overcharged.opmode.teleop2;

import java.util.ArrayList;


// Main Class
@Autonomous(name = "Pedro Path Test 2", group = "Autonomous")
public class autoPedroTest2 extends OpMode{

    private RobotMecanum robot;

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
    private Pose moveOutPose, dropperPose, ready2Score;
    private Pose initialspikeposeHeading = new Pose(0, 0, 180);

    private Pose startPose = new Pose(144-(63 + 72), 84, Math.PI);

    private Pose endPose = new Pose(1,1,0);

    //Other presets
    private Follower follower;
    private Path scoreSpikeMark, initialScoreOnBackdrop;
    private PathChain firstCycleToStack, firstCycleStackGrab, firstCycleScoreOnBackdrop, secondCycleToStack, secondCycleStackGrab, secondCycleScoreOnBackdrop;

    //Starting goal poses
    public void startingGoalPose() {
        switch (location) {
            case Left:
                spikeMarkGoalPose = new Pose(blueLeftSideLeftSpikeMark.getX() , blueLeftSideLeftSpikeMark.getY(), 0);
                moveOutPose = new Pose();
                break;
            default:
            case Middle:
                spikeMarkGoalPose = new Pose(blueLeftSideMiddleSpikeMark.getX()-10.5 ,blueLeftSideMiddleSpikeMark.getY(), 0);//0);
                moveOutPose = new Pose(blueLeftSideMiddleSpikeMark.getX()-20, blueLeftSideMiddleSpikeMark.getY(), 0);
                break;
            case Right:
                spikeMarkGoalPose = new Pose(blueLeftSideRightSpikeMark.getX() , blueLeftSideRightSpikeMark.getY(), 0);
                moveOutPose = new Pose();
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
                initialBackdropGoalPose = new Pose(blueMiddleBackdrop.getX()-0, blueMiddleBackdrop.getY() - 22,0);
                break;

            case Right:
                initialBackdropGoalPose = new Pose(blueMiddleBackdrop.getX() - 0.75, blueMiddleBackdrop.getY() - 2.75,0);
                break;

        }
        ready2Score = new Pose(blueMiddleBackdrop.getX(),blueMiddleBackdrop.getY()-28, 0);
    }


    // more presets
    //also init path sequence actions
    public void buildPaths() {
        switch (location) {
            case Left:

            default:
            case Middle:

            case Right:

        }
        //spikeMark
        //
        scoreSpikeMark = new Path(new BezierCurve(new Point(startPose), new Point(moveOutPose), new Point(spikeMarkGoalPose)));
        scoreSpikeMark.setConstantHeadingInterpolation(startPose.getHeading());
        scoreSpikeMark.setPathEndTimeoutConstraint(3);



        //first backdrop
        //
        initialScoreOnBackdrop = new Path(new BezierCurve(scoreSpikeMark.getLastControlPoint(), new Point(ready2Score), new Point(initialBackdropGoalPose)));
        initialScoreOnBackdrop.setConstantHeadingInterpolation(Math.toRadians(-90));
        initialScoreOnBackdrop.setPathEndTimeoutConstraint(3);

    }


    // Main pathing
    public void autoPath() {
        switch (pathState) {
            case 10: // starts following the spike mark detected
                follower.followPath(scoreSpikeMark);
                setPathState(11);
                break;
            case 11: // anti wall bump at start
                if (follower.getCurrentTValue() > 0.1) {
                    scoreSpikeMark.setLinearHeadingInterpolation(startPose.getHeading() - 0.1 * MathFunctions.getTurnDirection(startPose.getHeading(), scoreSpikeMark.getEndTangent().getTheta()) * MathFunctions.getSmallestAngleDifference(startPose.getHeading(), scoreSpikeMark.getEndTangent().getTheta()), scoreSpikeMark.getEndTangent().getTheta());
                    setPathState(12);
                }
                break;
            case 12: // pixel dropper
                if (!follower.isBusy()) {
                    robot.pixel.setLeftDump();
                    //waitFor(1000);
                    setPathState(13);
                }
                break;
            case 13: // score yellow path
                follower.followPath(initialScoreOnBackdrop);
                //setPathState(14);
                break;
            case 14: // raise slides & extend depo
                break;
            case 15: // retrieve depo
                break;
            case 16: //starting going for stack - 1st cycle
                break;


        }
    }

    // path setter
    public void setPathState(int state){
        pathState = state;
        //pathTimer.resetTimer();
        autoPath();
    }

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

    // initialization
    @Override
    public void init() {

        //robot = new RobotMecanum(this, true, false);


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
    @Override
    public void start() {
        startingGoalPose();
        backdropPoses();
        buildPaths();
        setPathState(10);
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

}
