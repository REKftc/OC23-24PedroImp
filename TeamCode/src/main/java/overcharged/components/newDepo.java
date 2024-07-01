package overcharged.components;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class newDepo {

    public OcServo frontClaw;
    public OcServo backClaw;
    public OcServo wrist;
    public OcServo arm;

    public OcServo angular;
    public AnalogInput armVolt;

    public static float FRONT_CLOSE = 119f;//71f;//28f;
    public static float FRONT_DUMP = 246f;//212f;//174f;//151f;
    //public static float FRONT_OPEN = 151f;
    public static float BACK_CLOSE = 209f;//138f;//127f;//219f;
    public static float BACK_DUMP = 81f;//0f;//4f;//46f;
    //public static float BACK_OPEN = 98f;

    //public static float WRIST_OPP_FLAT = 96f;//invert-68
    public static float WRIST_IN_VERT = 51f;//54f;
    public static float WRIST_OPP_VERT = WRIST_IN_VERT+141f; //192f;
    public static float WRIST_FLAT = WRIST_IN_VERT+72f; //123f;
    public static float WRIST_R_DIAG= WRIST_IN_VERT+118f;//169f;
    public static float WRIST_L_DIAG = WRIST_IN_VERT+24f;//75f;
    public static float ARM_IN = 221f;
    public static float ARM_OUT = 104f; //arm_out is more perpendicular to the ground so no bounce
    public static float ARM_PARALLEL = 119f; // arm_parallel is parallel to board
    public static float ARM_ANGULAR = 118f;
    public static float ARM_ANGULAR_RIGHT;

    public static float ANGULAR_STRAIGHT = 177f;//176f;
    public static float ANGULAR_LEFT = 214f;
    public static float ANGULAR_RIGHT = 147f;

    public newDepo(HardwareMap hardwareMap) {
        frontClaw = new OcServo(hardwareMap, "frontClaw", FRONT_DUMP);
        backClaw = new OcServo(hardwareMap, "backClaw", BACK_DUMP);
        wrist = new OcServo(hardwareMap, "wrist", WRIST_IN_VERT);
        arm = new OcServo(hardwareMap, "arm", ARM_IN);
        angular = new OcServo(hardwareMap, "angular", ANGULAR_STRAIGHT);
        armVolt = hardwareMap.get(AnalogInput.class, "armVolt");
    }
    public void setFrontClawPos(float pos){
        frontClaw.setPosition(pos);
    }

    public void setBackClawPos(float pos){
        backClaw.setPosition(pos);
    }

    public void setBothClawsOpen(){
        frontClaw.setPosition(FRONT_DUMP);
        backClaw.setPosition(BACK_DUMP);
    }

    public void setBothClawsClose(){
        frontClaw.setPosition(FRONT_CLOSE);
        backClaw.setPosition(BACK_CLOSE);
    }

    public void setWristPos(float pos){
        wrist.setPosition(pos);
    }

    public void setArmPos(float pos){
        arm.setPosition(pos);
    }

    public void setAngularPos(float pos){
        angular.setPosition(pos);
    }

    public void setAngularLeft(){ angular.setPosition(ANGULAR_LEFT);}

    public void setAngularRight(){ angular.setPosition(ANGULAR_RIGHT);}

    public double getArmVolt(){
        return armVolt.getVoltage(); //0 to 3.3
    }

    public void setDepoOutVert(){
        wrist.setPosition(WRIST_IN_VERT);
        arm.setPosition(ARM_OUT);
        angular.setPosition(ANGULAR_STRAIGHT);
    }

    public void setDepoIn(){
        wrist.setPosition(WRIST_IN_VERT);
        arm.setPosition(ARM_IN);
        angular.setPosition(ANGULAR_STRAIGHT);
    }
}


//package overcharged.components;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class newDepo {
//
//    public OcServo frontClaw;
//    public OcServo backClaw;
//    public OcServo arm;
//    public OcServo wrist;
//    public AnalogInput armVolt;
//    public OcServo angular;
//
//    public static float FRONT_CLOSE = 72f;//32f;//9f;
//    public static float FRONT_DUMP = 166f;//164f;//115f;
//    //public static float FRONT_OPEN = 151f;
//    public static float BACK_CLOSE = 133f;//212f;
//    public static float BACK_DUMP = 50f;//72f;
//    //public static float BACK_OPEN = 98f;
//    public static float ANGULAR_STRAIGHT = 173f;
//
//    public static float WRIST_IN_VERT = 146f;//53f;
//    public static float WRIST_OPP_VERT = 10f;//237f;
//    public static float WRIST_OPP_FLAT = 57f;//142f;
//    public static float WRIST_FLAT = 237f;//142f;
//    public static float WRIST_R_DIAG= 117f;//107f;
//    public static float WRIST_L_DIAG = 181f;//88f;
//
//
//    //public static float LEFT_IN = 133f;//131f;//127f;
//   // public static float RIGHT_IN = 18f;//22f;//18f;`
//
//    //public static float L_VERT = 166f;
//    public static float R_VERT_IN = 170f;//166f;
//    public static float L_VERT = 26f;
//    public static float R_FLAT = 235f;//238f;//118f;
//    public static float L_FLATDOWN = 95f;//177f;
//
//    public static float L_DIAG = 46f;//74f;//75f;
//    public static float R_DIAG = 137f;//133f;//134f;
//   // public static float L_N_DIAG = 162f;//166f;
//    public static float R_N_DIAG = 221f;//225f;
//    //public static float L_OPP_VERT = 192f;
//    public static float R_OPP_VERT = 251f;
//
//    public static float ARM_IN = 221f;//219f;
//    public static float ARM_OUT = 118f;//130f;
//
//    public newDepo(HardwareMap hardwareMap) {
//        frontClaw = new OcServo(hardwareMap, "frontClaw", FRONT_DUMP);
//        backClaw = new OcServo(hardwareMap, "backClaw", BACK_DUMP);
//        arm = new OcServo(hardwareMap, "arm", ARM_IN);
//        wrist = new OcServo(hardwareMap, "wrist", R_VERT_IN);
//        armVolt = hardwareMap.get(AnalogInput.class, "armVolt");
//        angular = new OcServo(hardwareMap, "angular", ANGULAR_STRAIGHT);
//    }
//    public void setFrontClawPos(float pos){
//        frontClaw.setPosition(pos);
//    }
//
//    public void setAngularPos(float pos){
//        frontClaw.setPosition(pos);
//    }
//
//    public void setBackClawPos(float pos){
//        backClaw.setPosition(pos);
//    }
//
//    public void setBothClawsOpen(){
//        frontClaw.setPosition(FRONT_DUMP);
//        backClaw.setPosition(BACK_DUMP);
//    }
//
//
//    public void setBothClawsClose(){
//        frontClaw.setPosition(FRONT_CLOSE);
//        backClaw.setPosition(BACK_CLOSE);
//    }
//
//    public void setWristPos(float pos){
//        arm.setPosition(pos);
//    }
//
//    public void setArmPos(float pos){
//        wrist.setPosition(pos);
//    }
//
//    public double getArmVolt(){
//        return armVolt.getVoltage(); //0 to 3.3
//    }
//
//    public void setBothDepoPosition(int rPos, int lPos){
//        wrist.setPosition(rPos);
//        arm.setPosition(lPos);
//    }
////

//    public void setDepoOutOppVert(){
//        wrist.setPosition(R_OPP_VERT);
//        arm.setPosition(ARM_OUT);
//        angular.setPosition(ANGULAR_STRAIGHT);
//    }
//
//    public void setDepoOutPDiag(){
//        wrist.setPosition(R_DIAG);
//        arm.setPosition(ARM_OUT);
//        angular.setPosition(ANGULAR_STRAIGHT);
//    }
//
//    public void setDepoOutNDiag(){
//        wrist.setPosition(R_N_DIAG);
//        arm.setPosition(ARM_OUT);
//        angular.setPosition(ANGULAR_STRAIGHT);
//    }
//
//    public void setDepoOutFlatL(){
//        wrist.setPosition(L_FLATDOWN);
//        arm.setPosition(ARM_OUT);
//        angular.setPosition(ANGULAR_STRAIGHT);
//    }
//
//    public void setDepoOutFlatR(){
//        wrist.setPosition(R_FLAT);
//        arm.setPosition(ARM_OUT);
//        angular.setPosition(ANGULAR_STRAIGHT);
//    }
//
//    public void setDepoIn(){
//        wrist.setPosition(R_VERT_IN);
//        arm.setPosition(ARM_IN);
//        angular.setPosition(ANGULAR_STRAIGHT);
//    }
//
//}
//
