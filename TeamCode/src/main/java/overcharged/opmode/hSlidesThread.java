package overcharged.opmode;

import static overcharged.config.RobotConstants.TAG_A;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.RobotMecanum;
import overcharged.components.SignalColors;
import overcharged.components.hslides;
import overcharged.config.RobotConstants;
import overcharged.linear.util.WaitLinear;

public class hSlidesThread implements Runnable {
    private RobotMecanum robot;
    private WaitLinear lp;
    private LinearOpMode opMode;

    double startencoder;
    long slideDownTime;
    float power = -0.85f;
    boolean stayIn = false;

    private SignalColors signalColors = SignalColors.Red;

    public hSlidesThread(boolean in, WaitLinear wl, LinearOpMode mode, RobotMecanum r) {
        // store parameter for later use
        lp = wl;
        opMode = mode;
        robot = r;
        stayIn = in;
    }

    public void run() {
        try {
            if (!opMode.isStopRequested()) {
                RobotLog.v(TAG_A, "Start raising slides");
                long startTime = System.currentTimeMillis();

                if(stayIn){
                    slidesIn(lp);
                }

                long TimeElapsed = System.currentTimeMillis() - startTime;
                RobotLog.ii(TAG_A, "Stop raising slides " + (System.currentTimeMillis() - startTime) + " milliseconds");
            }
        } catch (InterruptedException e) {
            RobotLog.ii(TAG_A, "Error: " + e.getStackTrace());
        }
    }

    public void slidesIn(WaitLinear lp) throws InterruptedException {
        long loopTime = System.currentTimeMillis();
        int safetyCounter = 0;
        while(stayIn && System.currentTimeMillis()-loopTime > 1000){
            if(!robot.hslides.switchSlideDown.isTouch()){
                while(!robot.hslides.switchSlideDown.isTouch()){
                    robot.hslides.hslidesR.setPower(-1);
                    robot.hslides.hslidesL.setPower(-1);
                }
                //robot.hslides.moveEncoderTo(hslides.START, -1);
                if(safetyCounter > 15){
                    robot.hslides.forceStop();
                    stayIn = false;
                }
                safetyCounter++;
            } else {
                safetyCounter = 0;
            }
            loopTime = System.currentTimeMillis();
        }
    }
}