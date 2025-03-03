package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Elevator extends SubsystemBase{

  private SparkMax m_motorLeftLead;
  private SparkMax m_motorRight;
  private SparkClosedLoopController m_PIDController;
  private RelativeEncoder m_encoder;
  private final SlewRateLimiter speedLimiter = new SlewRateLimiter(1.5);
  SparkMaxConfig globalConfig = new SparkMaxConfig();
  SparkMaxConfig m_motorLeftConfig = new SparkMaxConfig();
  SparkMaxConfig m_motorRightConfig = new SparkMaxConfig();

  private final double GEAR_RATIO;
  private final double PULLEY_DIAMETER_CM;
  private final double ENCODER_CPR;
  private double POSITION_CONVERSION_FACTOR;
  private double VELOCITY_CONVERSION_FACTOR;

  public enum ElevatorState{
    NOTRELEVATOR(0.0),
    L1ELEVATOR(50.0),
    L2ELEVATOR(100.0),
    L3ELEVATOR(150.0),
    L4ELEVATOR(200.0);
    
    private final double position;
    ElevatorState(double position) {
      this.position = position;
  }
  

  public double getPosition() {
      return position;
  }
  }

  private ElevatorState currentState = ElevatorState.NOTRELEVATOR;
  private static final double MAX_RPM = 2000; //NEO larda max hiz 5700


  public Elevator() {
    m_motorLeftLead = new SparkMax(
      20/*Constants.ELEVATOR_MOTOR_IDS[0]*/,
      MotorType.kBrushless
    );
    m_motorRight = new SparkMax(
      15/*Constants.ELEVATOR_MOTOR_IDS[1]*/,
      MotorType.kBrushless
    );

  globalConfig
    .smartCurrentLimit(50)
    .idleMode(IdleMode.kBrake) 
    .openLoopRampRate(.5)
    .voltageCompensation(12);

  m_motorLeftConfig
    .apply(globalConfig)
    .inverted(true);

  m_motorRightConfig
    .apply(globalConfig)
    .follow(m_motorLeftLead);

    m_PIDController = m_motorLeftLead.getClosedLoopController();

    m_encoder = m_motorLeftLead.getEncoder();
  
 // Dişli oranı ve kasnak çapına göre encoder dönüşüm faktörünü ayarlayın
  GEAR_RATIO = 10.0; // 10:1 dişli oranı
  PULLEY_DIAMETER_CM = 5.0; // Kasnak çapı (cm)
  ENCODER_CPR = 42.0; // NEO Encoder CPR

// Pozisyon dönüşüm faktörü (cm per tick)
POSITION_CONVERSION_FACTOR = 
    ((Math.PI * PULLEY_DIAMETER_CM) / (GEAR_RATIO * ENCODER_CPR));

// Hız dönüşüm faktörü (cm per saniye)
VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60.0;

m_motorLeftConfig.encoder
  .positionConversionFactor(POSITION_CONVERSION_FACTOR)
  .velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);


  m_motorLeftConfig.closedLoop
  .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
  .pid(Constants.ELEVATOR_PID[0],Constants.ELEVATOR_PID[1],Constants.ELEVATOR_PID[2]);

    m_motorLeftLead.configure(m_motorLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_motorRight.configure(m_motorRightConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    
    resetEncoder();
    
  }



  public void setElevatorState(ElevatorState state) {
    currentState = state;
    
    double targetPosition = currentState.getPosition();
    
    // PID Çıkışı için güvenli voltaj sınırlaması // Minimum -8V çıkış

    // PID Çıkışını sınırlıyoruz
    m_PIDController.setReference(targetPosition, SparkMax.ControlType.kPosition);

}




public void setRPM(double targetRPM) {
  // Hedef RPM güvenli sınırlar içinde mi?
  targetRPM = Math.max(-MAX_RPM, Math.min(targetRPM, MAX_RPM));

  // PID kontrolü ile RPM ayarı
  m_PIDController.setReference(targetRPM, ControlType.kVelocity);
}

    


  public double getPosition() {
    return m_encoder.getPosition();
  }

  private double lastSpeed = 0.0;
private final double SLEW_RATE = 0.1; // Maksimum hız değişim oranı

public void manuelMove(double speed){
  m_motorLeftLead.set(speedLimiter.calculate(speed));

}


  public double getVelocity() {
    return m_encoder.getVelocity();}

    public void resetEncoder() {
      m_encoder.setPosition(0);
      currentState = ElevatorState.NOTRELEVATOR; // Fiziksel olarak en aşağıda olduğunu varsayıyoruz
  }
  

  public void stop(){
    m_motorLeftLead.set(0);
  }

  private double heightToEncoderTicks(double height) {
    double conversionFactor = 10.0; // 1 cm ≈ 10 encoder tick (motorun dişli oranına bağlı)
    return height * conversionFactor;}


}