package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class CoralIntakeCommand extends Command {
    private final Intake intake;

    public CoralIntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.ToggleCoralIntake();
    }

    @Override
    public void end(boolean interrupted){
        
    }

    @Override
    public boolean isFinished() {
        return true;  // Butona basıldığında sadece bir kez çalışır
    }
}