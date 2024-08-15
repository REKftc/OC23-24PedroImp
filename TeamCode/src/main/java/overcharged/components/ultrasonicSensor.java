package overcharged.components;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ultrasonicSensor {
    public AnalogInput sensorBack;
    public AnalogInput sensorLeft;
    //public double leftVolt = sensorLeft.getVoltage();

   // public double backVolt = sensorBack.getVoltage();

    public ultrasonicSensor(HardwareMap hardwareMap) {
        sensorBack = hardwareMap.get(AnalogInput.class, "sensorBack");
        sensorLeft = hardwareMap.get(AnalogInput.class, "sensorLeft");
    }
    public float getSonarDistance(double volt){
        return (float) (((volt*6f*1024f/2.7f)-300f)/25.4f);

    }
    public float getLeftSonarDistance(){
        return getSonarDistance((float)sensorLeft.getVoltage());
    }
    public float getBackSonarDistance(){
        return getSonarDistance((float)sensorBack.getVoltage());
    }
    public double getBackVolt(){
        return sensorBack.getVoltage();
    }
    public double getLeftVolt(){
        return sensorLeft.getVoltage();
    }
    // y = 1.0028x - 0.8546
    public float getActualSonarDistance(double volt){
        return (float) ((volt*1.0028f)- 0.8287f);
    }
}