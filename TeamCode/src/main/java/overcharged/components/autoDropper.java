package overcharged.components;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class autoDropper {
   public OcServo dropper;
   public static final float INIT = 42f;
   public static float DOWN = 206f;
   public autoDropper(HardwareMap hardwareMap) {
      dropper = new OcServo(hardwareMap, "dropper", INIT);
   }
   public void setPosition(float pos){
      dropper.setPosition(pos);
   }

   public void setInit() { dropper.setPosition(INIT); }

   public void setOut() { dropper.setPosition(DOWN); }
}
