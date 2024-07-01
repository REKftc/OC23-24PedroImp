package overcharged.opmode;

import static overcharged.config.RobotConstants.TAG_A;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.RobotMecanum;
import overcharged.components.SignalColors;
import overcharged.config.RobotConstants;
import overcharged.linear.util.WaitLinear;

public class vSlidesThread implements Runnable {
    private RobotMecanum robot;
    private WaitLinear lp;
    private LinearOpMode opMode;

    double startencoder;
    long slideDownTime;
    float power = -0.85f;
    boolean raiseUp = false;

    private SignalColors signalColors = SignalColors.Red;

    public vSlidesThread(float p, boolean up, WaitLinear wl, LinearOpMode mode, RobotMecanum r) {
        // store parameter for later use
        lp = wl;
        opMode = mode;
        robot = r;
        power = -p;
        raiseUp = up;
    }

    public void run() {
        try {
            if (!opMode.isStopRequested()) {
                RobotLog.v(TAG_A, "Start raising slides");
                long startTime = System.currentTimeMillis();

                if(raiseUp==true){
                    slideUp(lp);
                } else{
                    slideDown(lp);
                }

                long TimeElapsed = System.currentTimeMillis() - startTime;
                RobotLog.ii(TAG_A, "Stop raising slides " + (System.currentTimeMillis() - startTime) + " milliseconds");
            }
        } catch (InterruptedException e) {
            RobotLog.ii(TAG_A, "Error: " + e.getStackTrace());
        }
    }

    public void slideUp(WaitLinear lp) throws InterruptedException {

    }

    private boolean slideReachedBottom() {
        return robot.vSlides.switchSlideDown.isTouch() && robot.vSlides.vSlides.getCurrentPosition() <= robot.vSlides.start;
    }

    public void slideDown(WaitLinear lp) throws InterruptedException {

        robot.vSlides.vSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideDownTime = System.currentTimeMillis();
        RobotLog.ii(RobotConstants.TAG_R, "reached bottom? " + robot.vSlides.switchSlideDown.isTouch() + " time elapsed " + (System.currentTimeMillis() - slideDownTime));
        while(!robot.vSlides.switchSlideDown.isTouch() && System.currentTimeMillis() - slideDownTime < 2000){
            RobotLog.ii(RobotConstants.TAG_R, "reached bottom? " + robot.vSlides.switchSlideDown.isTouch() + " power " + robot.vSlides.vSlides.getPower() + " time elapsed " + (System.currentTimeMillis() - slideDownTime));
            robot.vSlides.moveToBottom();
        }
        robot.vSlides.setPower(0);
        robot.vSlides.forcestop();
        robot.vSlides.reset(robot.vSlides.vSlides);


    }
}