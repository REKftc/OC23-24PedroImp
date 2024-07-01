package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class intakeBigTilt {
    public OcServo intakeBigTilt;
    public VoltageSensor intakeVolt;
    public static final float INIT = 175f;//230f;
    public static final float TRANSFER = 215f;//187f;//175f;
    public static final float FLAT = 171f;//158f;
    public static final float OUT = 69f;//62f;//52f;
    public static final float DUMP_EXTRA = 90f;
    public static final float FIFTH = 85f;//68f;
    public static final float FOURTH = FIFTH-5;//63f;
    public static final float THIRD = FOURTH-6;
    public static final float SECOND = THIRD-6;


    public intakeBigTilt(HardwareMap hardwareMap) {
        intakeBigTilt = new OcServo(hardwareMap, "intakeBigTilt", TRANSFER);
        intakeVolt = hardwareMap.voltageSensor.iterator().next();
    }
    public void setPosition(float pos){
        intakeBigTilt.setPosition(pos);
    }

    public void setInit() { intakeBigTilt.setPosition(INIT); }

    public void setTransfer() { intakeBigTilt.setPosition(TRANSFER); }

    public void setFlat() { intakeBigTilt.setPosition(FLAT); }

    public void setOut() { intakeBigTilt.setPosition(OUT); }

    public void getVoltage() { intakeVolt.getVoltage();}
}
