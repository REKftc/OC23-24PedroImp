package overcharged.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class OcEncoder {
    DcMotor ocEncoder;
    private double start;
    public double position = 0;

    int sign = 1;

    public static double TICK_TO_INCH = 242.5521332720485;

    public OcEncoder(String name, DcMotorSimple.Direction direction, HardwareMap hardwareMap){
        ocEncoder = hardwareMap.dcMotor.get(name);
        if(direction == DcMotorSimple.Direction.REVERSE) sign = -1;
        start = ocEncoder.getCurrentPosition();
    }

    private void update(){
        position = sign*(ocEncoder.getCurrentPosition() - start);
    }

    public double getPosition(){
        update();
        return position;
    }

    public void resetPosition(){
        start = ocEncoder.getCurrentPosition();
        update();
    }

    public double getRawPosition(){
        update();
        return position;
    }
}
