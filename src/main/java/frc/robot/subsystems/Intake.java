package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase {
    private SparkMax intakeMotor;
    private DigitalInput limitswitchcoral;
    private boolean isRunningCoralIntake = false;
    private LED m_led;


    public Intake(LED led) {
        intakeMotor = new SparkMax(Constants.INTAKE_MOTOR_ID,MotorType.kBrushless);
        limitswitchcoral = new DigitalInput(Constants.INTAKECORAL_LIMIT_SWITCH_ID);
        this.m_led = led;
    }

    public void ToggleCoralIntake(){
        if (isRunningCoralIntake) {
            stopIntake();
        } else if (!isCoralDetected()){
            startCoralIntake();
        }
    }

    

    public void startCoralIntake() {
        if (!isCoralDetected()) {
            intakeMotor.set(0.6);
            isRunningCoralIntake = true;
        } 
    }



    
    public void AlgIntake(double speed){
        System.out.println("AlgIntake Calisiyor");
        intakeMotor.set(-1);

    }



    public void stopIntake(){
        intakeMotor.set(0);
        isRunningCoralIntake = false;
    }
    
    
    public void CoralIOandAlgOuttake(double speed){
        System.out.println("CoralIOandAlgOuttake Calisiyor");
        intakeMotor.set(1);
    }


    public boolean isCoralDetected(){
        return !limitswitchcoral.get();
    }



    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Coral LS", isCoralDetected());
        if (isRunningCoralIntake && isCoralDetected()) {
            stopIntake();
        }
        if (isCoralDetected()) {
            m_led.setGreen(); // Coral içerideyse yeşil
        } else if (isRunningCoralIntake) {
            m_led.blinkRed(); // Coral alınırken kırmızı yanıp sönsün
        } else {
            m_led.setRed(); // Boştayken kırmızı yanar
        }
    }
    
    









    /*public class StartIntake extends Command {
        private final Intake intake;
    
        public StartIntake(Intake intake) {
            this.intake = intake;
            addRequirements(intake);
        }
    
        @Override
        public void initialize() {
            intake.CoralAnd(0.5); 
        }
        
        @Override
        public void end(boolean interrupted){
        intake.stopIntake();
    }
    
        @Override
        public boolean isFinished() {
            return true;  // Komut sona erer
        }
    }

    public class StopIntake extends Command {
        private final Intake intake;
    
        public StopIntake(Intake intake) {
            this.intake = intake;
            addRequirements(intake);
        }
    
        @Override
        public void initialize() {
            intake.stopIntake();  // Motoru durdur
        }
        
    
        @Override
        public boolean isFinished() {
            return true;  // Komut sona erer
        }
    }*/
}
