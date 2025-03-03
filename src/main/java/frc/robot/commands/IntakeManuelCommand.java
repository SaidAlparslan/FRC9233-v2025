package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeManuelCommand extends Command {
    private final Intake intake;
    private final double speed;
    private final boolean isAlgIntake;

    public IntakeManuelCommand(Intake intake, double speed, boolean tersmi) {
        this.intake = intake;
        this.speed = speed;
        this.isAlgIntake = tersmi;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (isAlgIntake){
            intake.AlgIntake(speed);
        }
        else{
            intake.CoralIOandAlgOuttake(speed);
        }
        
    }
    @Override
    public void end(boolean interrupted){
        intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false;  // Surekli halde olabilir
    }
}