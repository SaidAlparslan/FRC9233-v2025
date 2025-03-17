package frc.robot;

public class Constants {

    
    // Elevator Motor ve Encoder ID'leri
    public static final int[] ELEVATOR_MOTOR_IDS = {9, 12};// // Neo motorları
    public static final int ELEVATOR_LIMIT_SWITCH_ID = 1;
 // İvme kazancı


    
    // Intake Motor ID ve Limit Switch ID
    public static final int INTAKE_MOTOR_ID = 11;
    public static final int INTAKECORAL_LIMIT_SWITCH_ID = 0;//

    // Bumper Intake Motor ID ve Limit Switch ID
    public static final int[] INTAKE_ANGLE_MOTOR_ID = {19,20};
    public static final int INTAKE_ROLLER_MOTOR_ID = 13;
    public static final int INTAKE_LIMIT_SWITCH_ID = 5;//

    //
    public static final int ARM_MOTOR_ID = 10;
    
    // PID Değerleri
    public static final double[] ELEVATOR_PID = {0.1, 0.0, 0.1}; // // Örnek PID değerleri
    
}
