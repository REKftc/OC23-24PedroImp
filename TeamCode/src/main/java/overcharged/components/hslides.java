package overcharged.components;

import static overcharged.config.RobotConstants.TAG_H;
import static overcharged.config.RobotConstants.TAG_SL;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

import overcharged.config.RobotConstants;

public class hslides {

    public final OcMotorEx hslidesL;
    public final OcMotorEx hslidesR;
    public OcSwitch switchSlideDown;
    public final List<OcSwitch> switchs = new ArrayList<>();

    public double start;

    public static int autoLevel = 0;

    public static final int START = 0;
    public static final int PRESET1 = 142;
    public static final int OUT = 1000;

    public static double p = 18;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public hslides(HardwareMap hardwareMap) {
        hslidesR = new OcMotorEx(hardwareMap, "hslidesR", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        hslidesL = new OcMotorEx(hardwareMap, "hslidesL", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        hslidesR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hslidesL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hslidesR.setTargetPositionPIDFCoefficients(p, i, d, f);
        hslidesL.setTargetPositionPIDFCoefficients(p, i, d, f);

        String missing = "";
        int numberMissing = 0;
        OcMotorEx slideL = null;
        OcSwitch lswitch2 = null;
        try {
            lswitch2 = new OcSwitch(hardwareMap, "hlimitswitch", true);
            boolean isSwitchNull = switchSlideDown == null ? true : false;
            switchs.add(lswitch2);
            RobotLog.ii(TAG_H, "limitSwitch(isNull)? " + isSwitchNull);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R, "missing: limitSwitch " + e.getMessage());
            missing = missing + ", hlimitswitch";
            numberMissing++;
            boolean isSwitchNull = switchSlideDown == null ? true : false;
            RobotLog.ii(TAG_H, "limitSwitch(catch)? " + isSwitchNull);
        }
        this.switchSlideDown = lswitch2;
        start = hslidesR.getCurrentPosition();
        start = hslidesL.getCurrentPosition();

        RobotLog.ii(TAG_SL, "Initialized the Slide component numberMissing=" + numberMissing + " missing=" + missing);
    }

    public void outF() {
        hslidesR.setPower(1);
    }

    public void offF() {
        hslidesL.setPower(0);
    }

    public void inF() {
        hslidesR.setPower(-1f);
    }

    public void outB() {
        hslidesL.setPower(1);
    }

    public void offB() {
        hslidesR.setPower(0);
    }

    public void inB() {
        hslidesL.setPower(-1f);
    }

    public void moveEncoderTo(int pos, float power) {
        hslidesR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hslidesL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hslidesR.setTargetPosition(pos);
        hslidesL.setTargetPosition(pos);
        hslidesR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hslidesL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hslidesR.setPower(power);
        hslidesL.setPower(power);
    }

    public boolean slideIn() {
        return switchSlideDown.isTouch() && hslidesR.getCurrentPosition() <= start;
    }

    public void in(){
        hslidesL.setPower(-0.7f);
        hslidesR.setPower(-0.7f);
    }
    public void inAuto(){
        hslidesL.setPower(-0.9f);
        hslidesR.setPower(-0.9f);
    }

    public void setPower(float power)
    {
        int cnt = 1;
        //try {
        if (hslidesR != null) {
            RobotLog.ii(TAG_SL, "Set slide left motor power to " + power);
            hslidesR.setPower(power);
            if (power == 0f) {
                hslidesR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        } else {
            RobotLog.ii(TAG_SL, "Not setting power for motorL");
        }
        cnt = 2;
        if (hslidesL != null) {
            RobotLog.ii(TAG_SL, "Set slide right motor power to " + power);
            hslidesL.setPower(power);
            if (power == 0f) {
                hslidesL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        } else {
            RobotLog.ii(TAG_SL, "Not setting power for motorR");
        }
    }

    public void reset(OcMotorEx motor) {
        motor.setPower(0f);
        motor.setPower(0f);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.resetPosition();
    }
    public float getPower() {
        return hslidesR.getPower();
    }

    public float getPowerL() {
        return hslidesL.getPower();
    }


    public void forceStop() {
        //if (prev_state != State.STOP) {
        RobotLog.ii(TAG_SL, "Force stop the Slide component");
        hslidesR.setPower(0f);
        hslidesL.setPower(0f);
    }
}

