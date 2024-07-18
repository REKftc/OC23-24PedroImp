package overcharged.opmode;

// Imports
import static overcharged.config.RobotConstants.ROBOT_BACK_LENGTH;
import static overcharged.config.RobotConstants.ROBOT_FRONT_LENGTH;

import static java.lang.Thread.sleep;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
import overcharged.test.HSVPipeline;

import java.util.ArrayList;


// Main Class
public class autoPedroTest extends OpMode {

    private Timer pathTimer, opmodeTimer, scanTimer, distanceSensorUpdateTimer, distanceSensorDecimationTimer;

    private int pathState, distanceSensorDisconnectCycleCount, detectDistanceSensorDisconnect;

    private String navigation;


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


    private Pose spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleBackdropGoalPose, secondCycleStackPose, secondCycleBackdropGoalPose;
    private Pose startPose = new Pose(1, 1, 0);

    private Follower follower;
    private Path scoreSpikeMark, initialScoreOnBackdrop;
    private PathChain firstCycleToStack, firstCycleStackGrab, firstCycleScoreOnBackdrop, secondCycleToStack, secondCycleStackGrab, secondCycleScoreOnBackdrop;

    //private ArrayList<Boolean> distanceSensorDisconnects;
    /*
    public void setBackdropGoalPose() {
        switch (navigation) {
            default:
            case "left":
                spikeMarkGoalPose = new Pose(blueLeftSideLeftSpikeMark.getX() + 0.5, blueLeftSideLeftSpikeMark.getY(), Math.PI/2);
                initialBackdropGoalPose = new Pose(blueLeftBackdrop.getX() - 2, blueLeftBackdrop.getY()-ROBOT_BACK_LENGTH-2.5, Math.PI * 1.5);
                firstCycleBackdropGoalPose = new Pose(blueRightBackdrop.getX() - 1, blueRightBackdrop.getY()-ROBOT_BACK_LENGTH-1.25, Math.PI * 1.5);
                secondCycleBackdropGoalPose = new Pose(blueRightBackdrop.getX() - 1, blueRightBackdrop.getY()-ROBOT_BACK_LENGTH-1.25, Math.PI * 1.5);
                break;
            case "middle":
                spikeMarkGoalPose = new Pose(blueLeftSideMiddleSpikeMark.getX(), blueLeftSideMiddleSpikeMark.getY()+3, Math.PI/2);
                initialBackdropGoalPose = new Pose(blueMiddleBackdrop.getX() - 0.75, blueMiddleBackdrop.getY()-ROBOT_BACK_LENGTH-2.75,Math.PI * 1.5);
                firstCycleBackdropGoalPose = new Pose(blueRightBackdrop.getX()+0.5, blueRightBackdrop.getY()-ROBOT_BACK_LENGTH-1.5, Math.PI * 1.5);
                secondCycleBackdropGoalPose = new Pose(blueRightBackdrop.getX()+0.5, blueRightBackdrop.getY()-ROBOT_BACK_LENGTH-1.25, Math.PI * 1.5);
                break;
            case "right":
                spikeMarkGoalPose = new Pose(blueLeftSideRightSpikeMark.getX() + 2, blueLeftSideRightSpikeMark.getY()+0.5, Math.PI/2);
                initialBackdropGoalPose = new Pose(blueRightBackdrop.getX()+1,blueRightBackdrop.getY()-ROBOT_BACK_LENGTH-2.75, Math.PI * 1.5);
                firstCycleBackdropGoalPose = new Pose(blueMiddleBackdrop.getX() - 2.5, blueRightBackdrop.getY()-ROBOT_BACK_LENGTH-1.25, Math.PI * 1.5);
                secondCycleBackdropGoalPose = new Pose(blueRightBackdrop.getX() - 2, blueRightBackdrop.getY()-ROBOT_BACK_LENGTH-1.25, Math.PI * 1.5);
                break;
        }
    }
    */

    // Preset paths
    public void buildPaths() {
        Point scoreSpikeMarkMidPoint;
        double scoreSpikeMarkMidToSpikeDistance;
        switch (navigation) {
            default:
            case "left":
                scoreSpikeMarkMidPoint = new Point(144 - 131.5, 82, Point.CARTESIAN);
                break;
            case "middle":
                scoreSpikeMarkMidPoint = new Point(144 - 129.5, 106, Point.CARTESIAN);
                break;
            case "right":
                scoreSpikeMarkMidPoint = new Point(144 - 122.5, 99, Point.CARTESIAN);
                break;
        }
        scoreSpikeMarkMidToSpikeDistance = MathFunctions.distance(spikeMarkGoalPose, scoreSpikeMarkMidPoint);
        scoreSpikeMark = new Path(new BezierCurve(new Point(startPose), scoreSpikeMarkMidPoint, new Point(spikeMarkGoalPose.getX() + MathFunctions.getSign(scoreSpikeMarkMidPoint.getX() - spikeMarkGoalPose.getX()) * Math.abs(scoreSpikeMarkMidPoint.getX() - spikeMarkGoalPose.getX()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, spikeMarkGoalPose.getY() + MathFunctions.getSign(scoreSpikeMarkMidPoint.getY() - spikeMarkGoalPose.getY()) * Math.abs(scoreSpikeMarkMidPoint.getY() - spikeMarkGoalPose.getY()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, Point.CARTESIAN)));
        scoreSpikeMark.setConstantHeadingInterpolation(startPose.getHeading());
        scoreSpikeMark.setPathEndTimeoutConstraint(3);

        switch (navigation) {
            default:
            case "left":
                initialScoreOnBackdrop = new Path(new BezierCurve(scoreSpikeMark.getLastControlPoint(), new Point(144 - 135, 98, Point.CARTESIAN), new Point(initialBackdropGoalPose.getX(), 106, Point.CARTESIAN), new Point(initialBackdropGoalPose)));
                break;
            case "middle":
                initialScoreOnBackdrop = new Path(new BezierLine(scoreSpikeMark.getLastControlPoint(), new Point(initialBackdropGoalPose)));
                break;
            case "right":
                initialScoreOnBackdrop = new Path(new BezierCurve(scoreSpikeMark.getLastControlPoint(), new Point(scoreSpikeMark.getLastControlPoint().getX(), 98, Point.CARTESIAN), new Point(initialBackdropGoalPose.getX(), 109.5, Point.CARTESIAN), new Point(initialBackdropGoalPose)));
                break;
        }
        initialScoreOnBackdrop.setConstantHeadingInterpolation(Math.PI * 1.5);
        initialScoreOnBackdrop.setLinearHeadingInterpolation(scoreSpikeMark.getEndTangent().getTheta(), Math.PI * 1.5, 0.5);
        initialScoreOnBackdrop.setPathEndTimeoutConstraint(2.5);

        switch (navigation) {
            default:
            case "left":
                firstCycleStackPose = new Pose(blueInnerStack.getX() - 3, blueInnerStack.getY() + ROBOT_FRONT_LENGTH - 0.5, Math.PI * 1.5 + Math.toRadians(-1));
                secondCycleStackPose = new Pose(blueInnerStack.getX() - 5, blueInnerStack.getY() + ROBOT_FRONT_LENGTH - 1.5, Math.PI * 1.5 + Math.toRadians(-2));
                break;
            case "middle":
                firstCycleStackPose = new Pose(blueInnerStack.getX() - 3, blueInnerStack.getY() + ROBOT_FRONT_LENGTH - 2.5, Math.PI * 1.5 + Math.toRadians(-1.5));
                secondCycleStackPose = new Pose(blueInnerStack.getX() - 5, blueInnerStack.getY() + ROBOT_FRONT_LENGTH - 2.75, Math.PI * 1.5 + Math.toRadians(-2.5));
                break;
            case "right":
                firstCycleStackPose = new Pose(blueInnerStack.getX() - 3, blueInnerStack.getY() + ROBOT_FRONT_LENGTH - 3, Math.PI * 1.5 + Math.toRadians(-2));
                secondCycleStackPose = new Pose(blueInnerStack.getX() - 5, blueInnerStack.getY() + ROBOT_FRONT_LENGTH - 3, Math.PI * 1.5 + Math.toRadians(-5));
                break;
        }
        /*
        firstCycleToStack = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(initialBackdropGoalPose), new Point(144-76.5, 106, Point.CARTESIAN), new Point(firstCycleStackPose.getX()+2, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .addPath(new BezierLine(new Point(firstCycleStackPose.getX()+2, 79, Point.CARTESIAN), new Point(firstCycleStackPose.getX(), 23, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        firstCycleStackGrab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstCycleStackPose.getX()+0.0001, 32, Point.CARTESIAN), new Point(firstCycleStackPose)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .build();

        firstCycleScoreOnBackdrop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstCycleStackPose), new Point(firstCycleStackPose.getX()+2, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .addPath(new BezierCurve(new Point(firstCycleStackPose.getX()+2, 79, Point.CARTESIAN), new Point(144-76.5, 106, Point.CARTESIAN), new Point(firstCycleBackdropGoalPose)))
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .setPathEndTimeoutConstraint(2.5)
                .build();

        secondCycleToStack = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(firstCycleBackdropGoalPose), new Point(144-76.5, 106, Point.CARTESIAN), new Point(secondCycleStackPose.getX()+2, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(secondCycleStackPose.getHeading())
                .addPath(new BezierLine(new Point(secondCycleStackPose.getX()+2, 79, Point.CARTESIAN), new Point(secondCycleStackPose.getX(), 23, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(secondCycleStackPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        secondCycleStackGrab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondCycleStackPose.getX()+0.0001, 32, Point.CARTESIAN), new Point(secondCycleStackPose)))
                .setConstantHeadingInterpolation(secondCycleStackPose.getHeading())
                .build();

        secondCycleScoreOnBackdrop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondCycleStackPose), new Point(secondCycleStackPose.getX()+2, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(secondCycleStackPose.getHeading())
                .addPath(new BezierCurve(new Point(secondCycleStackPose.getX()+2, 79, Point.CARTESIAN), new Point(144-76.5, 106, Point.CARTESIAN), new Point(secondCycleBackdropGoalPose)))
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .setPathEndTimeoutConstraint(2.5)
                .build(); */
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10: // starts following the first path - score spike
                follower.followPath(scoreSpikeMark);
                //setPathState(11);
                break;
            //case 11: // move away from wall

        }
    }

    public void setPathState(int state){
            pathState = state;
            //pathTimer.resetTimer();
            autonomousPathUpdate();
        }


    @Override
    public void loop() {
        //updateDistanceSensorDisconnects();
        follower.update();
        //teleop2.autonomousControlUpdate();

        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        //twoPersonDrive.telemetry();
        //telemetry.update();
    }



    @Override
    public void init() {
        scanTimer = new Timer();
    }

    @Override
    public void init_loop() {
        if (scanTimer.getElapsedTime() > 750) {
            //navigation = new HSVPipeline();
            //telemetry.addData("Navigation:", navigation);
            telemetry.update();
            /*
            scanTimer.resetTimer();
        } else if (scanTimer.getElapsedTime() > 700){
            visionPortal.setProcessorEnabled(teamPropPipeline, true);
        } else {
            visionPortal.setProcessorEnabled(teamPropPipeline, false);
        } */
        }
    }



    @Override
    public void start() {
        //visionPortal.stopStreaming();
        //twoPersonDrive.frameTimer.resetTimer();
        //setBackdropGoalPose();
        buildPaths();
        //opmodeTimer.resetTimer();
        setPathState(10);
    }

    @Override
    public void stop() {
    }


    }

