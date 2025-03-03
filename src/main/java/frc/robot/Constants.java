package frc.robot;

public class Constants {

    
    // Elevator Motor ve Encoder ID'leri
    public static final int[] ELEVATOR_MOTOR_IDS = {19, 18}; // Neo motorları
    public static final int[] ELEVATOR_ENCODER_IDS = {11,12};
    public static final int ELEVATOR_LIMIT_SWITCH_ID = 1;
    
    // Intake Motor ID ve Limit Switch ID
    public static final int INTAKE_MOTOR_ID = 10;
    public static final int INTAKECORAL_LIMIT_SWITCH_ID = 0;

    //
    public static final int ARM_MOTOR_ID = 9;
    
    // PID Değerleri
    public static final double[] ELEVATOR_PID = {0.1, 0.0, 0.1};  // Örnek PID değerleri
    
}
