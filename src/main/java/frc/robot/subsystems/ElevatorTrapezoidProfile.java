package frc.robot.subsystems;

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
    public enum ElevatorTrapezoidState {
        NOTRL1(0.0), 
        L2(70.0), 
        L3(100.0),
        L4(150.0), 
        NET(190.0);
        private final double targetHeight;
        ElevatorTrapezoidState(double targetHeight) { this.targetHeight = targetHeight; }
        public double getTargetHeight() { return targetHeight; }
    }

    private final SparkMax m_motorLeftLead;
    private final SparkMax m_motorRight;
    private final RelativeEncoder m_encoder;
    private final DigitalInput limitSwitchelevator = new DigitalInput(Constants.ELEVATOR_LIMIT_SWITCH_ID);
    private final DIOSim limitSwitchSim = new DIOSim(limitSwitchelevator);

    private static final double GEAR_RATIO = 10.0;
    private static final double PULLEY_DIAMETER_CM = 5.0;
    private static final double ENCODER_CPR = 42.0;
    private static final double POSITION_CONVERSION_FACTOR = ((Math.PI * PULLEY_DIAMETER_CM) / (GEAR_RATIO * ENCODER_CPR));
    private static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60.0;
    private static final double MIN_HEIGHT = 0.0;
    private static final double MAX_HEIGHT = 100.0;

    private final SparkMaxConfig globalConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_motorLeftConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_motorRightConfig = new SparkMaxConfig();

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(50.0, 100.0);
    private final ProfiledPIDController controller = new ProfiledPIDController(0.1, 0.0, 0.1, constraints);

    private ElevatorTrapezoidState currentState = ElevatorTrapezoidState.NOTRL1;

    public ElevatorTrapezoidProfile() {
        m_motorLeftLead = new SparkMax(Constants.ELEVATOR_MOTOR_IDS[0], MotorType.kBrushless);
        m_motorRight = new SparkMax(Constants.ELEVATOR_MOTOR_IDS[1], MotorType.kBrushless);

        globalConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake).openLoopRampRate(0.5).voltageCompensation(12);
        m_motorLeftConfig.apply(globalConfig).inverted(true);
        m_motorLeftConfig.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR).velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
        m_motorRightConfig.apply(globalConfig).follow(m_motorLeftLead);

        m_encoder = m_motorLeftLead.getEncoder();
        m_motorLeftLead.configure(m_motorLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motorRight.configure(m_motorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        resetEncoder();
    }

    public void resetEncoder() { m_encoder.setPosition(0); }

    public void setHeight(double targetHeight) {
        double clampedHeight = Math.max(MIN_HEIGHT, Math.min(MAX_HEIGHT, targetHeight));
        controller.setGoal(clampedHeight);
    }

    public void setState(ElevatorTrapezoidState state) {
        this.currentState = state;
        setHeight(state.getTargetHeight());
        if (RobotBase.isSimulation()) {
            SmartDashboard.putString("Elevator Target State", state.name());
        }
    }

    public double getCurrentPosition() { return m_encoder.getPosition(); }

    @Override
    public void periodic() {
        double currentPosition = m_encoder.getPosition();
        double output = Math.max(-1.0, Math.min(1.0, controller.calculate(currentPosition)));
        m_motorLeftLead.set(output);
        if (DriverStation.isDisabled() && !limitSwitchelevator.get()) {
            resetEncoder();
            currentState = ElevatorTrapezoidState.NOTRL1;
        }
        if (RobotBase.isSimulation()) {
            SmartDashboard.putNumber("Elevator Current Position", currentPosition);
            String dashboardTargetState = SmartDashboard.getString("Elevator Target State", currentState.name());
            try {
                ElevatorTrapezoidState newState = ElevatorTrapezoidState.valueOf(dashboardTargetState);
                if (newState != currentState) {
                    setState(newState);
                }
            } catch (IllegalArgumentException e) {
                SmartDashboard.putString("Elevator Target State", currentState.name());
            }
        }
    }

    @Override
    public void simulationPeriodic() {
        double appliedOutput = m_motorLeftLead.get();
        double simulatedPosition = getCurrentPosition() + (appliedOutput * 0.1);
        m_encoder.setPosition(simulatedPosition);

        if (simulatedPosition <= MIN_HEIGHT) {
            limitSwitchSim.setValue(true);
            resetEncoder();
        } else {
            limitSwitchSim.setValue(false);
        }
    }
}
