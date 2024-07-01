package overcharged.opmode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import overcharged.components.Button;
import overcharged.components.RobotMecanum;


@Config
@TeleOp(name="testTele", group="Test")
public class testTele extends OpMode {
    RobotMecanum robot;
    boolean down = false;
    @Override
    public void init() {
        robot = new RobotMecanum(this, false, false);
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.setBulkReadManual();
    }
    @Override
    public void loop() {
        robot.clearBulkCache();
        long timestamp = System.currentTimeMillis();
        telemetry.addData("dropper pos: ", robot.dropper.dropper.getPosition());
        telemetry.addData("down: ", down);
        if(gamepad1.a && Button.BTN_ALIGNER.canPress(timestamp)){
            if(down) {
                robot.dropper.setInit();
                down = false;
            }
            else if(!down){
                robot.dropper.setOut();
                down = true;
            }
        }
    }
}
