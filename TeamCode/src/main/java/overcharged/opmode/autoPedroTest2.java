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

    //cam
    private int pathState;
    HSVPipeline detector;
    OpenCvWebcam webcam;
    propLocation location = propLocation.Middle;
    EasyOpenCVExample.RingDeterminationPipeline pipeline;

    // all spike mark locations
    private Pose redLeftSideLeftSpikeMark = new Pose(36 + 72, -47.5 + 72);
    private Pose redLeftSideMiddleSpikeMark = new Pose(24.5 + 72, -36 + 72);
    private Pose redLeftSideRightSpikeMark = new Pose(36 + 72, -24.5 + 72);
    private Pose redRightSideLeftSpikeMark = new Pose(36 + 72, 0.5 + 72);
    private Pose redRightSideMiddleSpikeMark = new Pose(24.5 + 72, 12 + 72);
    private Pose redRightSideRightSpikeMark = new Pose(36 + 72, 23.5 + 72);
    private Pose blueLeftSideLeftSpikeMark = new Pose(-36 + 72, 23.5 + 72);
    private Pose blueLeftSideMiddleSpikeMark = new Pose(-24.5 + 72, 12 + 72);
    private Pose blueLeftSideRightSpikeMark = new Pose(-36 + 72, 0.5 + 72);
    private Pose blueRightSideLeftSpikeMark = new Pose(-36 + 72, -24.5 + 72);
    private Pose blueRightSideMiddleSpikeMark = new Pose(-24.5 + 72, -36 + 72);
    private Pose blueRightSideRightSpikeMark = new Pose(-36 + 72, -47.5 + 72);

    // backdrop april tag locations
    private Pose blueLeftBackdrop = new Pose(-42.875 + 72, 60.75 + 72);
    private Pose blueMiddleBackdrop = new Pose(-36.75 + 72, 60.75 + 72);
    private Pose blueRightBackdrop = new Pose(-30.75 + 72, 60.75 + 72);
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


    //presets
    private Pose spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleBackdropGoalPose, secondCycleStackPose, secondCycleBackdropGoalPose;
    private Pose moveOutPose, dropperPose, ready2Score;
    private Pose startPose = new Pose(144-(63 + 72), 84, Math.PI);
    private Pose initialspikeposeHeading = new Pose(0, 0, 180);

    private Follower follower;
    private Path scoreSpikeMark, initialScoreOnBackdrop;
    private PathChain firstCycleToStack, firstCycleStackGrab, firstCycleScoreOnBackdrop, secondCycleToStack, secondCycleStackGrab, secondCycleScoreOnBackdrop;

    public void startingGoalPose() {
        switch (location) {
            case Left:
                spikeMarkGoalPose = new Pose(blueLeftSideLeftSpikeMark.getX() , blueLeftSideLeftSpikeMark.getY(), 0);
                moveOutPose = new Pose();
                break;
            default:
            case Middle:
                spikeMarkGoalPose = new Pose(blueLeftSideMiddleSpikeMark.getX()-14 ,blueLeftSideMiddleSpikeMark.getY(), 0);//0);
                moveOutPose = new Pose(blueLeftSideMiddleSpikeMark.getX()-28, blueLeftSideMiddleSpikeMark.getY(), 0);
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
                initialBackdropGoalPose = new Pose(blueMiddleBackdrop.getX() - 0.75, blueMiddleBackdrop.getY() - 10,0);
                break;

            case Right:
                initialBackdropGoalPose = new Pose(blueMiddleBackdrop.getX() - 0.75, blueMiddleBackdrop.getY() - 2.75,0);
                break;

        }
        ready2Score = new Pose(blueMiddleBackdrop.getX(),blueMiddleBackdrop.getY()-20, 0);
    }


    // more presets
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
        scoreSpikeMark.setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(-75));//Math.PI*3/2);
        //scoreSpikeMark.setConstantHeadingInterpolation(startPose.getHeading());
        scoreSpikeMark.setPathEndTimeoutConstraint(3);


        //first backdrop
        //
        initialScoreOnBackdrop = new Path(new BezierCurve(scoreSpikeMark.getLastControlPoint(), new Point(ready2Score), new Point(initialBackdropGoalPose)));
        initialScoreOnBackdrop.setConstantHeadingInterpolation(Math.toRadians(-75));
        //initialScoreOnBackdrop.setLinearHeadingInterpolation(scoreSpikeMark.getEndTangent().getTheta(), Math.PI * 1.5, 0.5);
        initialScoreOnBackdrop.setPathEndTimeoutConstraint(3);

    }


    // Main pathing
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10: // starts following the first path - score purple
                follower.followPath(scoreSpikeMark);

                setPathState(11);
                break;
            case 11: // score yellow path
                follower.followPath(initialScoreOnBackdrop);
                //setPathState(12);
                break;


        }
    }

    // Sets the path
    public void setPathState(int state){
        pathState = state;
        //pathTimer.resetTimer();
        autonomousPathUpdate();
    }

    public void startDistanceSensorDisconnectDetection(int state) {

    }

    @Override
    public void loop() {
        follower.update();
    }

    // RAHHHH
    @Override
    public void init() {

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
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        telemetry.addData("Prop Location", location);
        telemetry.update();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        startingGoalPose();
        backdropPoses();
        buildPaths();
        setPathState(10);
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
}
