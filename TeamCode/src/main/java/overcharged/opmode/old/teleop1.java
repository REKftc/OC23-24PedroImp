package overcharged.opmode.old;


import static overcharged.config.RobotConstants.TAG_SL;
import static overcharged.config.RobotConstants.TAG_T;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.Button;
import overcharged.components.RobotMecanum;
import overcharged.components.hslides;


@Disabled
@TeleOp(name="teleop1", group="Teleop")
public class teleop1 extends OpMode {
    RobotMecanum robot;
    boolean down = false;
    long startTime;
    long intakeDoorDelay;
    long depoTiltDelay;
    long shootDelay;
    boolean intakeDDelay = false;
    double slowPower = 1;
    private final static float SLOW_POWER_MULT = 0.65f;
    boolean isSlow = true;
    SlowMode slowMode = SlowMode.SLOW;
    IntakeMode intakeMode = IntakeMode.OFF;
    FlatButtonState flatbuttonState = FlatButtonState.NO2;
    TransferButtonState transferButtonState = TransferButtonState.NO2;
    DepoMode depoMode = DepoMode.OPEN2;
    boolean iSmallTilt = false;
    boolean dTilt = false;
    boolean dOpen = true;
    boolean iOpen = false;
    boolean slideGoBottom = false;
    boolean firstLoop = true;
    boolean rPixelOpen = false;
    boolean lPixelOpen = false;
    boolean dTiltIn = false;
    boolean shooting = false;
    boolean isLocked = true;
    long depoTiltInDelay;
    boolean depoTiltOutDelay = false;
    boolean intakeTransfer = false;
    boolean intakeOutDelay = false;
    long intakeOutTime;
    boolean stayIn = true;
    boolean isOut = false;

    boolean hSlideGoBottom = false;

    boolean hSlideisOut = false;


    public enum SlowMode {
        OVERRIDE,
        SLOW;
    }

    public enum DepoMode {
        ClOSED,
        OPEN1,
        OPEN2,
        OPEN3;
    }

    public enum IntakeMode {
        IN,
        OUT,
        OFF;
    }

    public enum FlatButtonState {
        PRESSED,
        PRESSING,
        NO,
        NO2;
    }

    public enum TransferButtonState {
        PRESSED,
        PRESSING,
        NO,
        NO2;
    }

    @Override
    public void init() {
        try {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            robot = new RobotMecanum(this, false, false);
            startTime = System.currentTimeMillis();
            robot.setBulkReadManual();
            //robot.vSlides.vSlidesB.setTargetPositionPIDFCoefficients(21,0,0,0);
        } catch (Exception e){
            RobotLog.ee(TAG_T, "Teleop init failed: " + e.getMessage());
            telemetry.addData("Init Failed", e.getMessage());
            telemetry.update();
        }
    }
    @Override
    public void loop() {
        if(firstLoop) {
            robot.intakeSmallTilt.setOut();
            robot.intakeBigTilt.setOut();
            robot.intakeDoor.setClosed();
            iOpen = false;
            firstLoop = false;
            robot.hang.setIn();
        }
        robot.clearBulkCache();
        long timestamp = System.currentTimeMillis();

        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = ((y + x + rx) / denominator)*slowPower;
        double backLeftPower = ((y - x + rx) / denominator)*slowPower;
        double frontRightPower = ((y - x - rx) / denominator)*slowPower;
        double backRightPower = ((y + x - rx) / denominator)*slowPower;

        robot.driveLeftFront.setPower(frontLeftPower);
        robot.driveLeftBack.setPower(backLeftPower);
        robot.driveRightFront.setPower(frontRightPower);
        robot.driveRightBack.setPower(backRightPower);

        /**
         * Turn on slow mode for drive train
         */
        if (gamepad1.a && Button.BTN_SLOW_MODE.canPress(timestamp)) {
            isSlow = true;
        } else {
            isSlow = false;
        }

        if(slowMode == SlowMode.SLOW) {
            if (isSlow) {
                slowPower = 0.4f;
                y *= SLOW_POWER_MULT;
                x *= SLOW_POWER_MULT;
                rx *= SLOW_POWER_MULT;
            } else {
                slowPower = 1;
            }
        }

        if(stayIn && !robot.hslides.switchSlideDown.isTouch()){
            robot.hslides.moveEncoderTo(hslides.START, 1);
        }
        //Intake in
        if(gamepad1.right_trigger > 0.9 && Button.INTAKE.canPress(timestamp)) {//gamepad1.right_bumper && Button.INTAKE.canPress(timestamp)){
            if (intakeMode == IntakeMode.OFF || intakeMode == IntakeMode.OUT) {
                robot.intakeSmallTilt.setOut();
                robot.intakeBigTilt.setOut();
                robot.intakeDoor.setClosed();
                iOpen = false;
                robot.intake.in();
                intakeMode = IntakeMode.IN;
            }
            else{
                robot.intake.off();
                intakeMode = IntakeMode.OFF;
            }
        }

        //Intake out
        if(gamepad1.left_trigger > 0.9 && Button.INTAKEOUT.canPress(timestamp)){//bumper && Button.INTAKEOUT.canPress(timestamp)){
            if(intakeMode == IntakeMode.IN || intakeMode == IntakeMode.OFF) {
                robot.intake.out();
                intakeMode = IntakeMode.OUT;
            }
            else {
                robot.intake.off();
                intakeMode = IntakeMode.OFF;
            }
        }

        /*//Intake tilt
        if(gamepad2.right_bumper && Button.TRANSFER.canPress(timestamp)){
            if(!iSmallTilt){
                robot.intakeSmallTilt.setTransfer();
                robot.intakeBigTilt.setTransfer();
                robot.intakeDoor.setOpen();
                robot.depoDoor.setOpen();
                iSmallTilt = true;
            } else {
                robot.intakeSmallTilt.setOut();
                robot.intakeBigTilt.setOut();
                robot.intakeDoor.setClosed();
                robot.depoDoor.setClosed();
                iSmallTilt = false;
            }
        }*/

        if(gamepad2.right_bumper && Button.TRANSFER.canPress(timestamp)){//bumper && Button.INTAKEOUT.canPress(timestamp)){
            stayIn = true;
            if(!intakeTransfer) {
                robot.intakeBigTilt.setTransfer();
                intakeDDelay = true;
                intakeDoorDelay = System.currentTimeMillis();
                //robot.intakeDoor.setOpen();
                robot.depoDoor.setOpen2();
                depoMode = DepoMode.OPEN2;
                intakeTransfer = true;
            }
            else {
                robot.intakeSmallTilt.setOut();
                intakeOutDelay = true;
                intakeOutTime = System.currentTimeMillis();
                robot.intakeBigTilt.setOut();
                robot.intakeDoor.setClosed();
                robot.depoDoor.setClosed();
                depoMode = DepoMode.ClOSED;
                intakeDDelay = false;
                iOpen = false;
                intakeTransfer = false;
            }
        }

        if(intakeDDelay && System.currentTimeMillis()-intakeDoorDelay > 100){
            robot.intakeSmallTilt.setTransfer();
            //intakeDDelay = false;
        }

        if(intakeDDelay && System.currentTimeMillis()-intakeDoorDelay > 650){
            robot.intakeDoor.setOpen();
            iOpen = true;
            intakeDDelay = false;
        }

        /*if(intakeOutDelay && System.currentTimeMillis()-intakeOutTime > 100){
            robot.intakeBigTilt.setOut();
            intakeOutDelay = false;
        }*/

        /*//Hold for transfer position
        if(gamepad2.right_bumper){
            if(transferButtonState == TransferButtonState.NO2) {
                transferButtonState = TransferButtonState.PRESSED;
            }
            else {
                transferButtonState = TransferButtonState.PRESSING;
            }
        } else {
            if(transferButtonState == TransferButtonState.PRESSING)
                transferButtonState = TransferButtonState.NO;
            else
                transferButtonState = TransferButtonState.NO2;
        }

        if(transferButtonState == TransferButtonState.PRESSED){
            //robot.intakeSmallTilt.setTransfer();
            robot.intakeBigTilt.setTransfer();
            intakeDDelay = true;
            intakeDoorDelay = System.currentTimeMillis();
            //robot.intakeDoor.setOpen();
            robot.depoDoor.setOpen2();
            depoMode = DepoMode.OPEN2;
        } else if(transferButtonState == TransferButtonState.NO){
            robot.intakeSmallTilt.setOut();
            robot.intakeBigTilt.setOut();
            robot.intakeDoor.setClosed();
            robot.depoDoor.setClosed();
            depoMode = DepoMode.ClOSED;
            intakeDDelay = false;
            iOpen = false;
        }*/



        //Hold for flat position
        if(gamepad2.right_trigger > 0.9){
            if(flatbuttonState == FlatButtonState.NO2) {
                flatbuttonState = FlatButtonState.PRESSED;
            }
            else {
                flatbuttonState = FlatButtonState.PRESSING;
            }
        } else {
            if(flatbuttonState == FlatButtonState.PRESSING)
                flatbuttonState = FlatButtonState.NO;
            else
                flatbuttonState = FlatButtonState.NO2;
        }

        if(flatbuttonState == FlatButtonState.PRESSED){
            robot.intakeSmallTilt.setFlat();
            robot.intakeBigTilt.setFlat();
            robot.intakeDoor.setClosed();
            iOpen = false;
        } else if(flatbuttonState == FlatButtonState.NO){
            robot.intakeSmallTilt.setOut();
            robot.intakeBigTilt.setOut();
            robot.intakeDoor.setClosed();
            robot.depoDoor.setClosed();
            iOpen = false;
            depoMode = DepoMode.ClOSED;
            intakeTransfer = false;
        }

        //IntakeDoor
        if(gamepad2.b && Button.INTAKEDOOR.canPress(timestamp)){
            if(!iOpen){
                robot.intakeDoor.setOpen();
                iOpen = true;
            }
            else{
                robot.intakeDoor.setClosed();
                iOpen = false;
            }
        }

        //DepoTilt
        if(gamepad2.left_bumper && Button.DEPOTILT.canPress(timestamp)){
            if(!dTilt){
                robot.depoTilt.setOut();
                robot.depoDoor.setClosed();
                dTilt = true;
                depoMode = DepoMode.ClOSED;
            }
            else{
                robot.vSlides.moveEncoderTo((int)(robot.vSlides.vSlides.getCurrentPosition())+80, 1);
                depoTiltDelay = System.currentTimeMillis();
                dTiltIn = true;
            }
        }
        if(dTiltIn && System.currentTimeMillis() - depoTiltDelay > 120){
            robot.depoDoor.setClosed();
            depoMode = DepoMode.ClOSED;

            robot.depoTilt.setIn();
            dOpen = false;
            dTilt = false;
            dTiltIn = false;
        }

        //DepoDoor all the way open
        if(gamepad1.left_bumper && Button.DEPODOOR.canPress(timestamp)){
            if(depoMode == DepoMode.ClOSED || depoMode == DepoMode.OPEN1){
                robot.depoDoor.setOpen2();
                depoMode = DepoMode.OPEN2;
            }
            else{
                robot.depoDoor.setClosed();
                depoMode = DepoMode.ClOSED;
            }
        }

        if(gamepad1.right_bumper && Button.DEPODOOR.canPress(timestamp)){
            if(depoMode == DepoMode.ClOSED){
                robot.depoDoor.setOpen1();
                depoMode = DepoMode.OPEN1;
            } else if(depoMode == DepoMode.OPEN1){
                robot.depoDoor.setOpen2();
                depoMode = DepoMode.OPEN2;
            } else if(depoMode == DepoMode.OPEN2){
                robot.depoDoor.setClosed();
                depoMode = DepoMode.ClOSED;
            }
        }
//      UNCOMMENT THIS LATER
        if(gamepad2.y && Button.SLIGHT_UP.canPress(timestamp)){
            if(robot.vSlides.vSlides.getCurrentPosition() < 460){
                robot.vSlides.moveEncoderTo((int)(robot.vSlides.vSlides.getCurrentPosition())+65, 1);
            }
        }

        if(gamepad2.a && Button.SLIGHT_DOWN.canPress(timestamp)){
            if(robot.vSlides.vSlides.getCurrentPosition() > 55){
                robot.vSlides.moveEncoderTo((int)(robot.vSlides.vSlides.getCurrentPosition())-50, 1);
            }
        }



//        if (gamepad2.y && Button.BTN_SLIDE_OUT.canPress(timestamp) && !hSlideisOut) {
//            hSlideisOut = true;
//            robot.intake.off();
//            intakeMode = IntakeMode.OFF;
//            robot.depoDoor.setClosed();
//            depoMode = DepoMode.ClOSED;
//            robot.hslides.moveEncoderTo(robot.hslides.OUT, 1);
//        }

//        if (gamepad2.a && Button.BTN_SLIDE_OUT.canPress(timestamp) && hSlideisOut) {
//            hSlideisOut = false;
//            robot.intake.off();
//            intakeMode = IntakeMode.OFF;
//            robot.depoDoor.setClosed();
//            depoMode = DepoMode.ClOSED;
//            robot.intake.in();
//              robot.hslides.in();
//            hSlideGoBottom = true;
//        }


//        if (hSlideGoBottom) {
//            if (robot.hslides.slideIn()) {
//                robot.hslides.forceStop();
//                robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                hSlideGoBottom = false;
//                robot.intake.in();
//                intakeMode = IntakeMode.OFF;
//                RobotLog.ii(TAG_SL, "Force stopped");
//            }
//            else {
//                robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.hslides.in();
//            }
//        }

        /*if (!robot.hslides.slideIn()) {
            robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.hslides.in();
        } else {
            robot.hslides.forceStop();
            robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//              robot.intake.in();
//              intakeMode = IntakeMode.OFF;
        }*/

//        if (!robot.hslides.slideIn() && hSlideGoBottom) {// && robot.vSlides.getCurrentPosition() > robot.vSlides.start){//!robot.vSlides.slideReachedBottom()){
//            robot.hslides.hslides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.hslides.in();
//            RobotLog.ii(TAG_SL, "Going down");
//        } else if (robot.hslides.slideIn() && hSlideGoBottom) {
//            robot.hslides.forceStop();
//            robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            hSlideGoBottom = false;
//            robot.intake.in();
//            intakeMode = IntakeMode.IN;
//            RobotLog.ii(TAG_SL, "Force stopped");
//        }

//        if (!robot.hslides.slideIn() && hSlideGoBottom) {
//            hSlideGoBottom = false;
//            robot.hslides.hslides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.hslides.forceStop();
//            robot.intake.in();
//            intakeMode = IntakeMode.IN;
//        }

        // vSlides down
        if((gamepad2.left_trigger > 0.9 || gamepad1.dpad_down) && Button.BTN_SLIDE_DOWN.canPress(timestamp)){
            if(robot.vSlides.vSlides.getCurrentPosition() < robot.vSlides.level4-20){
                robot.vSlides.moveEncoderTo(robot.vSlides.vSlides.getCurrentPosition()+120,1);
            }
            robot.intake.in();
            slideGoBottom = true;
            //robot.depoTilt.setIn();
            robot.depoDoor.setOpen2();
            depoMode = DepoMode.OPEN2;
            dTilt = false;
            depoTiltInDelay = System.currentTimeMillis();
        }
        //Slide height 1
        if(gamepad2.dpad_left && Button.BTN_LEVEL1.canPress(timestamp)){
            depoTiltOutDelay = true;
            robot.intake.off();
            intakeMode = IntakeMode.OFF;
            robot.depoDoor.setClosed();
            depoMode = DepoMode.ClOSED;
            robot.intakeDoor.setClosed();
            iOpen = false;
            robot.vSlides.moveEncoderTo(robot.vSlides.level1, 1);
        }
        //Slide height 2
        if(gamepad2.dpad_down && Button.BTN_LEVEL2.canPress(timestamp)){
            depoTiltOutDelay = true;
            robot.intake.off();
            intakeMode = IntakeMode.OFF;
            robot.depoDoor.setClosed();
            depoMode = DepoMode.ClOSED;
            robot.intakeDoor.setClosed();
            iOpen = false;
            robot.vSlides.moveEncoderTo(robot.vSlides.level2, 1);
        }
        //Slide height 3
        if(gamepad2.dpad_right && Button.BTN_LEVEL3.canPress(timestamp)){
            depoTiltOutDelay = true;
            robot.intake.off();
            intakeMode = IntakeMode.OFF;
            robot.depoDoor.setClosed();
            depoMode = DepoMode.ClOSED;
            robot.intakeDoor.setClosed();
            iOpen = false;
            robot.vSlides.moveEncoderTo(robot.vSlides.level3, 1);
        }
        //Slide height 4
        if(gamepad2.dpad_up && Button.BTN_LEVEL4.canPress(timestamp)){
            depoTiltOutDelay = true;
            robot.intake.off();
            intakeMode = IntakeMode.OFF;
            robot.depoDoor.setClosed();
            depoMode = DepoMode.ClOSED;
            robot.intakeDoor.setClosed();
            iOpen = false;
            robot.vSlides.moveEncoderTo(robot.vSlides.level4, 1);
        }

        if(depoTiltOutDelay && robot.vSlides.vSlides.getCurrentPosition() > robot.vSlides.level1 - 10 ){
            robot.depoTilt.setOut();
            dTilt = true;
            depoTiltOutDelay = false;
        }

        if(slideGoBottom)  {
            if(System.currentTimeMillis() - depoTiltInDelay > 500){//(robot.vSlides.vSlidesB.getCurrentPosition() > robot.vSlides.level2+ 20 && System.currentTimeMillis() - depoTiltInDelay > 100)
                //|| (robot.vSlides.vSlidesB.getCurrentPosition() <= robot.vSlides.level2 + 20 && System.currentTimeMillis() - depoTiltInDelay > 200)) {
                robot.depoDoor.setClosed();
                depoMode = DepoMode.ClOSED;
                robot.depoTilt.setIn();
                dTilt = false;
            }
        }

        if(slideGoBottom)  {
            if(System.currentTimeMillis() - depoTiltInDelay > 650){//(robot.vSlides.vSlidesB.getCurrentPosition() > robot.vSlides.level2+ 20 && System.currentTimeMillis() - depoTiltInDelay > 100)
                //|| (robot.vSlides.vSlidesB.getCurrentPosition() <= robot.vSlides.level2 + 20 && System.currentTimeMillis() - depoTiltInDelay > 200)) {
                slideBottom();
            }
        }

        if(gamepad1.y && Button.NOPOWER.canPress(timestamp)){
            slideGoBottom = false;
            robot.vSlides.stopMotors();
            robot.vSlides.forcestop();
            robot.vSlides.vSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if(gamepad1.x && Button.LEFTPIXEL.canPress(timestamp)){
            if(lPixelOpen){
                lPixelOpen = false;
                robot.pixel.setLeftPixelIn();
            } else if(!lPixelOpen){
                lPixelOpen = true;
                robot.pixel.setLeftDump();
            }
        }

        if(gamepad1.b && Button.RIGHTPIXEL.canPress(timestamp)){
            if(rPixelOpen){
                rPixelOpen = false;
                robot.pixel.setRightPixelIn();
            } else if (!rPixelOpen){
                rPixelOpen = true;
                robot.pixel.setRightDump();
            }
        }

        if(gamepad1.a && Button.DRONESHOOTER.canPress(timestamp)){
            if(!shooting){
                robot.droneShooter.setShoot();
                shooting = true;
            } else{
                robot.droneShooter.setInit();
                shooting = false;
            }
        }

        if(gamepad2.x && Button.BTN_LATCH_READY.canPress(timestamp)){
            if(isLocked) {
                robot.hang.setHang();
                isLocked = false;
            } else {
                robot.hang.setIn();
                isLocked = true;
            }
        }

        if(gamepad2.left_stick_y > 0.1 && !robot.vSlides.slideReachedBottom()){
            robot.vSlides.vSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.vSlides.vSlides.setPower(-gamepad2.left_stick_y/2);
        }

        if(gamepad2.left_stick_y < -0.1 && robot.vSlides.vSlides.getCurrentPosition() < 480){
            robot.vSlides.vSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.vSlides.vSlides.setPower(-gamepad2.left_stick_y/2);
        }

        if(gamepad2.right_stick_button && Button.BTN_HORIZONTAL.canPress(timestamp)){
            if(isOut){
                stayIn = true;
                isOut = false;
            } else{
                stayIn = false;
                robot.hslides.moveEncoderTo(400, 1);
                isOut = true;
            }
        }

        telemetry.addData("vSlides encoder: ", robot.vSlides.vSlides.getCurrentPosition());
        telemetry.addData("vSlide power: ", robot.vSlides.vSlides.getPower());
        telemetry.addData("v limit switch: ", robot.vSlides.switchSlideDown.isTouch());
        telemetry.addData("h limit switch: ", robot.hslides.switchSlideDown.isTouch());
        telemetry.addData("test:", robot.hslides.hslidesR.getPower());
        telemetry.addData("test2: ", robot.hslides.hslidesR.getCurrentPosition());
        telemetry.addData("reach bottom: ", !robot.vSlides.slideReachedBottom());
        telemetry.addData("driveLF", robot.driveLeftFront.getCurrentPosition());
        telemetry.addData("driveLB", robot.driveLeftBack.getCurrentPosition());
        telemetry.addData("driveRF", robot.driveRightFront.getCurrentPosition());
        telemetry.addData("intake", robot.intake.intake.getCurrentPosition());
        telemetry.addData("driveRB", robot.driveRightBack.getCurrentPosition());
        telemetry.addData("hslideOut", robot.hslides.slideIn());
        telemetry.addData("hslidePower", robot.hslides.getPower());
        telemetry.addData("hSlidePos", robot.hslides.hslidesR.getCurrentPosition());
        telemetry.update();
    }
    public void slideBottom() {
        if (!robot.vSlides.slideReachedBottom()) {
            robot.vSlides.vSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.vSlides.vSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.vSlides.down();
            RobotLog.ii(TAG_SL, "Going down");
        } else {
            robot.vSlides.forcestop();
            robot.vSlides.vSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideGoBottom = false;
            robot.intake.in();
            intakeMode = IntakeMode.IN;
            RobotLog.ii(TAG_SL, "Force stopped");
        }
    }
}