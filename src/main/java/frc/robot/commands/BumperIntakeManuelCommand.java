package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BumperIntake;

public class BumperIntakeManuelCommand extends Command {
    private final BumperIntake bumperIntake;

    public BumperIntakeManuelCommand(BumperIntake bumperIntake) {
        this.bumperIntake = bumperIntake;
        addRequirements(bumperIntake);
    }

    @Override
    public void initialize() {
        System.out.println("BumperIntake Manuel Mod Basladi");
    }

    @Override
    public void execute() {
        double manualSpeed = SmartDashboard.getNumber("Bumper Intake Angle Speed", 0.0); // Dashboard'dan hız al
        bumperIntake.setAngleMotorSpeed(manualSpeed); // Motoru manuel hızla döndür
    }

    @Override
    public void end(boolean interrupted) {
        bumperIntake.stopAngleMotor(); // Buton bırakıldığında motoru durdur
        System.out.println("BumperIntake Manuel Mod Bitti");
    }

    @Override
    public boolean isFinished() {
        return false; // Buton basılı olduğu sürece çalışacak
    }
}
