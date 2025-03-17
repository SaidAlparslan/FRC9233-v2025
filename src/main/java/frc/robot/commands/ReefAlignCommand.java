package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ReefAlignCommand extends Command {
    private final CommandSwerveDrivetrain swerve;
    private final LimelightHelpers bune;
    private final int reefnumber;

    public ReefAlignCommand(int reef,CommandSwerveDrivetrain swerve,LimelightHelpers bune) {
        this.reefnumber = reef;
        this.swerve=swerve;
        this.bune=bune;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        if (reefnumber==1){
            //bunu bunu yap
        }
        if (reefnumber==2){
            //bunu bunu yap
        }
        if (reefnumber==3){
            //bunu bunu yap
        }
        if (reefnumber==4){
            //bunu bunu yap
        }
        if (reefnumber==5){
            //bunu bunu yap
        }
        if (reefnumber==6){
            //bunu bunu yap
        }
        if (reefnumber==7){
            //bunu bunu yap
        }
        if (reefnumber==8){
            //bunu bunu yap
        }
        if (reefnumber==9){
            //bunu bunu yap
        }
        if (reefnumber==10){
            //bunu bunu yap
        }
        if (reefnumber==11){
            //bunu bunu yap
        }
        if (reefnumber==12){
            //bunu bunu yap
        }
    }

    public boolean isAligned(){
        return true;//BİŞİLER BİŞİLER
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return isAligned(); // Buton basılı olduğu sürece çalışacak
    }
}
