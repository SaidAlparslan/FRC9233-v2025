package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class TurnToAngleCommand extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private double goal;
    private boolean isRelative;
    private double startAngle;
    private double targetAngle;

    private final double tolerance = 0.01 * Math.PI; // ~3.6 deg
    
    // PID değerleri kalibre edilecek
    private final PIDController yawPidController = new PIDController(0, 0, 0);

    public TurnToAngleCommand(CommandSwerveDrivetrain drivetrain, double angle, boolean isRelative) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.goal = angle;
        this.isRelative = isRelative;

        yawPidController.setTolerance(tolerance);
    }

    @Override
    public void initialize() {
        startAngle = drivetrain.getRotation3d().getZ();

        if (isRelative) {
            targetAngle = startAngle + goal;
        } else {
            targetAngle = goal;
        }
    }

    @Override
    public void execute() {
        double currentAngle = drivetrain.getRotation3d().getZ();
        drivetrain.setControl(new SwerveRequest.FieldCentric().withRotationalRate(yawPidController.calculate(currentAngle, targetAngle))
        );
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.setControl(new SwerveRequest.Idle());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drivetrain.getRotation3d().getZ() - targetAngle) < tolerance;
    }
}
