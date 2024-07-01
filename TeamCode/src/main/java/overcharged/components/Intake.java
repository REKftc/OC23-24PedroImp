package overcharged.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public DcMotor intake;

    public Intake(HardwareMap hardwareMap){
        intake = hardwareMap.dcMotor.get("intake");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void in(){
        intake.setPower(1);
    }

    public void slowIn(){
        intake.setPower(0.7f);
    }


    public void off(){
        intake.setPower(0);
    }

    public void out(){
        intake.setPower(-0.7);
    }
}
