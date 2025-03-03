package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final SparkMax armMotor;
    private final RelativeEncoder armEncoder;
    private final ProfiledPIDController pidController;
    private boolean manualOverride = false;
    private ArmState targetState;

    //  ArmState Enum Tanımlandı
    public enum ArmState {
        NOTRINTAKEARM(0.0),
        L1_L2_L3ARM(45.0),
        L4ARM(60.0),
        NETARM(80.0),
        PROCESSORARM(180.0);


        private final double position; // Derece cinsinden konum

        ArmState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    // Encoder dönüşüm faktörleri
    private static final double GEAR_RATIO = 10.0; // 10:1 dişli oranı
    private static final double POSITION_CONVERSION_FACTOR = 1.0 / 36.0; // 1 derece = 0.02778 rotations// BUNLARI DİŞLİYE GÖRE FLN AYARLİCAZ
    private static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60.0; // Dakikadan saniyeye

    // Trapezoid Profil Sınırları
    private final TrapezoidProfile.Constraints constraints =
            new TrapezoidProfile.Constraints(2.5, 5.0); // 2.5 rotations/s, 5.0 rotations/s²

    public Arm() {
        armMotor = new SparkMax(Constants.ARM_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        
        config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        config.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR)
                      .velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);

        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armEncoder = armMotor.getEncoder();
        armEncoder.setPosition(0);
        
        // Trapezoid Profile PID Kontrolcüsü
        pidController = new ProfiledPIDController(0.5, 0.0, 0.1, constraints);
        pidController.setTolerance(1.0); // 1 derece hata payı

        targetState = ArmState.NOTRINTAKEARM;
    }

    // Hedef pozisyonu belirle
    public void setTargetState(ArmState state) {
        manualOverride = false;
        targetState = state;
        pidController.setGoal(state.getPosition()); // Trapezoid Profile hedefi ayarla
    }

    public ArmState getTargetState() {
        return targetState;
    }

    public double getCurrentPosition() {
        return armEncoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Arm State", targetState.name());
        SmartDashboard.putNumber("Arm Encoder (Degrees)", getCurrentPosition());
        SmartDashboard.putNumber("Arm Target Position", targetState.getPosition());

        if (!manualOverride) {
            double currentPosition = getCurrentPosition();
            double pidOutput = pidController.calculate(currentPosition);

            // Çıkışı -1.0 ile 1.0 arasında sınırlayalım
            pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));

            armMotor.set(pidOutput);
        }
    }

    // Manuel hareket
    public void manualMove(double speed) {
        manualOverride = true;
        armMotor.set(speed);
    }

    // Motoru durdur
    public void stop() {
        manualOverride = false;
        armMotor.set(0);
    }
}
