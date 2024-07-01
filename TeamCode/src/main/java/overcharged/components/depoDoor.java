package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class depoDoor {
    public OcServo depoDoor;
    public static final float CLOSED = 143f;
    public static float OPEN1 = 153f;
    public static float OPEN2 = 170f;

    public depoDoor(HardwareMap hardwareMap) {
        depoDoor = new OcServo(hardwareMap, "depoDoor", OPEN2);
    }
    public void setPosition(float pos){
        depoDoor.setPosition(pos);
    }

    public void setClosed() { depoDoor.setPosition(CLOSED); }

    public void setOpen1() { depoDoor.setPosition(OPEN1); }

    public void setOpen2() { depoDoor.setPosition(OPEN2); }
}