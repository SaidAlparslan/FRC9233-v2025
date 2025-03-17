package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorTrapezoidProfile extends SubsystemBase {
    
    // Elevator konumları
    public enum ElevatorTrapezoidState {
        NOTRL1(-1.0), 
        L2(-5.0), 
        L3(-16.0),
        L4(-22.0);

        private final double targetHeight;
        ElevatorTrapezoidState(double targetHeight) { this.targetHeight = targetHeight; }
        public double getTargetHeight() { return targetHeight; }
    }

    // Motorlar ve encoder
    private final SparkMax m_motorLeftLead;
    private final SparkMax m_motorRight;
    private final RelativeEncoder m_encoder;
    
    // Limit switch
    private final DigitalInput limitSwitchelevator = new DigitalInput(Constants.ELEVATOR_LIMIT_SWITCH_ID);
    private final DIOSim limitSwitchSim = new DIOSim(limitSwitchelevator);

    // Elevator mekanik özellikler
    private static final double GEAR_RATIO = 30.0;  
    private static final double PULLEY_DIAMETER_CM = 5.0;
    private static final double POSITION_CONVERSION_FACTOR = ((Math.PI * PULLEY_DIAMETER_CM) / (GEAR_RATIO));
    private static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60.0;
    private static final double MIN_HEIGHT = -26.0;
    private static final double MAX_HEIGHT = 0.0;

    // Motor konfigürasyonu
    private final SparkMaxConfig globalConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_motorLeftConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_motorRightConfig = new SparkMaxConfig();

    // Trapezoid profil ve PID
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(20, 30);
    private final ProfiledPIDController controller = new ProfiledPIDController(0.05, 0.0, 0.2, constraints);

    // **FEEDFORWARD EKLENDİ**
    //private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0.2, 0.5, 1.2, 0.02); 

    private ElevatorTrapezoidState currentState = ElevatorTrapezoidState.NOTRL1;

    public ElevatorTrapezoidProfile() {
        m_motorLeftLead = new SparkMax(Constants.ELEVATOR_MOTOR_IDS[1], MotorType.kBrushless);
        m_motorRight = new SparkMax(Constants.ELEVATOR_MOTOR_IDS[0], MotorType.kBrushless);

        globalConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).openLoopRampRate(0.2).voltageCompensation(12);
        m_motorLeftConfig.apply(globalConfig).inverted(false);
        m_motorLeftConfig.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR).velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
        m_motorRightConfig.apply(globalConfig).inverted(false).follow(m_motorLeftLead);

        m_encoder = m_motorLeftLead.getEncoder();
        m_motorLeftLead.configure(m_motorLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motorRight.configure(m_motorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        resetEncoder();
    }

    public void resetEncoder() { 
        m_encoder.setPosition(0); 
    }

    public void setHeight(double targetHeight) {
        double clampedHeight = Math.max(MIN_HEIGHT, Math.min(MAX_HEIGHT, targetHeight));
        controller.setGoal(clampedHeight);
    }

    public void setState(ElevatorTrapezoidState state) {
        this.currentState = state;
        setHeight(state.getTargetHeight());
    }

    public double getCurrentPosition() { 
        return m_encoder.getPosition(); 
    }

    public void stop() {
        m_motorLeftLead.set(0);
        //m_motorRight.set(0);
    }

    public void calistir(double percent){
        m_motorLeftLead.set(percent);
        //m_motorRight.set(percent);
    }
    
    

    @Override
    public void periodic() {
        double currentPosition = m_encoder.getPosition();

        // PID ve Feedforward hesaplaması
        /*double pidOutput = -controller.calculate(currentPosition);
        double ffOutput = -0.05;
        double output = pidOutput + ffOutput;  

        // Motor çıkışını sınırla
        output = Math.max(-1.0, Math.min(1.0, output));

        m_motorLeftLead.set(output);

        if (DriverStation.isDisabled() && !limitSwitchelevator.get()) {
            resetEncoder();
            currentState = ElevatorTrapezoidState.NOTRL1;
        }*/
        SmartDashboard.putNumber("Elevator Current Position", currentPosition);
    //SmartDashboard.putString("Elevator Target Position", getTargetState);
        
    }

    @Override
    public void simulationPeriodic() {
        /*double appliedOutput = m_motorLeftLead.get();
        double simulatedPosition = getCurrentPosition() + (appliedOutput * 0.5);
        m_encoder.setPosition(simulatedPosition);

        if (simulatedPosition <= MIN_HEIGHT) {
            limitSwitchSim.setValue(true);
            resetEncoder();
        } else {
            limitSwitchSim.setValue(false);
        }*/
    }
}
