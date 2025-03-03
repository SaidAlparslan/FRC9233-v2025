package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class StopIntake extends Command {
    private final Intake intake;

    public StopIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        System.out.println("Intake: Duruyor");
        intake.stopIntake();
        
    }
    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished() {
        return true;  // bi kere
    }
}