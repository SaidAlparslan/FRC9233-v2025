package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class StartIntake extends Command {
    private final Intake intake;

    public StartIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.CoralIOandAlgOuttake(0.5);
        System.out.println("Intake: Calisiyor");
    }
    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished() {
        return true;  // surekli degil
    }
}