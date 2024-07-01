package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class pixelDropper {
    public OcServo leftPixelDropper;
    public OcServo rightPixelDropper;
    public OcServo pixel;
    public static float LEFT_P_IN = 169f;
    public static float LEFT_DUMP = 111f;
    public static float RIGHT_P_IN = 88f;
    public static float RIGHT_DUMP = 148f;

    public static float MIDDLE = 105f;
    public static float LEFT_OUT = 42f;
    public static float RIGHT_OUT = 169f;
    public static float LEFT_IN = RIGHT_OUT;
    public static float RIGHT_IN = LEFT_OUT;

    public pixelDropper(HardwareMap hardwareMap) {
        //leftPixelDropper = new OcServo(hardwareMap, "leftPixel", LEFT_IN);
        //rightPixelDropper = new OcServo(hardwareMap, "rightPixel", RIGHT_IN);
        pixel = new OcServo(hardwareMap, "pixel", MIDDLE);
    }
    public void setLeftPosition(float pos){
        leftPixelDropper.setPosition(pos);
    }

    public void setLeftPixelIn() { leftPixelDropper.setPosition(LEFT_P_IN); }

    public void setLeftDump() { leftPixelDropper.setPosition(LEFT_DUMP); }

    public void setRightPosition(float pos){
        rightPixelDropper.setPosition(pos);
    }

    public void setRightPixelIn() { rightPixelDropper.setPosition(RIGHT_P_IN); }

    public void setRightDump() { rightPixelDropper.setPosition(RIGHT_DUMP); }


    public void setPos(float pos){pixel.setPosition(pos);}

    public void setBothIn() { pixel.setPosition(MIDDLE);}

    public void setRightIn() { pixel.setPosition(RIGHT_IN);}

    public void setLeftIn() { pixel.setPosition(LEFT_IN);}

    public void setLeftOut() { pixel.setPosition(LEFT_OUT);}

    public void setRightOut() { pixel.setPosition(RIGHT_OUT);}

}
