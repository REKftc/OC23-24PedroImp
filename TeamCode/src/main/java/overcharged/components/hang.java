package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class hang {
    public OcServo Hang;
    public static float IN = 130f;//162f;
    public static float HANG = 96f;//98f;//102f;//208f;

    public hang(HardwareMap hardwareMap, boolean isRight) {
        Hang = new OcServo(hardwareMap, "Hang", IN);
    }
    public void setPosition(float pos){Hang.setPosition(pos);
    }

    public void setIn() {
        Hang.setPosition(IN);
    }

    public void setHang() {
        Hang.setPosition(HANG);
    }

}
