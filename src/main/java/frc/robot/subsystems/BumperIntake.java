package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BumperIntake extends SubsystemBase {
    private final SparkMax intakeAngleMotor;
    private final SparkMax intakeAngleMotor2;
    private final SparkMax intakeRollerMotor;
    private final RelativeEncoder angleEncoder;
    private final DigitalInput bumperlimitSwitch;
    private final ProfiledPIDController anglePIDController;

    public enum IntakeAngleState {
        NOTR(0.0),      // Kapalı pozisyon 
        SHOOT(45.0),   // Top alma pozisyonu
        INTAKE(90.0);    // Fırlatma pozisyonu

        private final double angle;
        IntakeAngleState(double angle) { this.angle = angle; }
        public double getAngle() { return angle; }
    }

    private IntakeAngleState currentAngleState = IntakeAngleState.NOTR;

    private final SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig angleMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig angleMotorConfig2 = new SparkMaxConfig();

    // Konversiyon Faktörleri
    private static final double GEAR_RATIO = 48/34; // Örnek dişli oranı
    private static final double POSITION_CONVERSION_FACTOR = 360.0 / GEAR_RATIO; // Encoder rotations -> degrees// tam tersi de olabilir aslında
    private static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60.0; // RPM -> degrees/s

    // Trapezoid Profil Sınırları
    private final TrapezoidProfile.Constraints angleConstraints =
            new TrapezoidProfile.Constraints(100.0, 200.0); // Max hız: 100°/s, ivme: 200°/s²

    public BumperIntake() {
        intakeAngleMotor = new SparkMax(Constants.INTAKE_ANGLE_MOTOR_ID[0], MotorType.kBrushless);
        intakeAngleMotor2 = new SparkMax(Constants.INTAKE_ANGLE_MOTOR_ID[1], MotorType.kBrushed);
        intakeRollerMotor = new SparkMax(Constants.INTAKE_ROLLER_MOTOR_ID, MotorType.kBrushed);
        bumperlimitSwitch = new DigitalInput(Constants.INTAKE_LIMIT_SWITCH_ID);

        
        angleMotorConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        angleMotorConfig2.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40).follow(intakeAngleMotor);
        angleMotorConfig.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR)
                                .velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
        intakeAngleMotor2.configure(angleMotorConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeAngleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeRollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        angleEncoder = intakeAngleMotor.getEncoder();
        angleEncoder.setPosition(0); // İlk başta sıfırla

        // Açıyı kontrol eden Trapezoid PID
        anglePIDController = new ProfiledPIDController(0.8, 0.0, 0.2, angleConstraints);
        anglePIDController.setTolerance(2.0); // 2 derece hata payı
    }

    public void setAngleState(IntakeAngleState state) {
        currentAngleState = state;
        anglePIDController.setGoal(state.getAngle());
    }

    public void setRollerSpeed(double speed) {
        intakeRollerMotor.set(speed);
    }

    public void setAngleMotorSpeed(double speed){
        intakeAngleMotor.set(speed);
    }

    public void stopRoller() {
        intakeRollerMotor.set(0);
    }

    public void stopAngleMotor(){
        intakeAngleMotor.set(0);
    }

    public double getCurrentAngle() {
        return angleEncoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Intake State", currentAngleState.name());
        SmartDashboard.putNumber("Intake Angle", getCurrentAngle());

        // Açıyı PID ile kontrol et
        /*double pidOutput = anglePIDController.calculate(getCurrentAngle());
        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput)); // Çıkışı sınırla
        intakeAngleMotor.set(pidOutput);

        // Limit switch sıfırlama
        if (!bumperlimitSwitch.get()) {
            angleEncoder.setPosition(0);
            currentAngleState = IntakeAngleState.NOTR;
        }*/
    }
}
