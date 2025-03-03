package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmManuelCommand extends Command {
    private final Arm arm;
    private final double speed;

    /**
     * ArmCommand, arm'ı belirli bir pozisyona getirmek için kullanılır.
     *
     * @param arm             Arm alt sistemi nesnesi
     * @param speed  Hedef pozisyon (encoder birimine göre, örneğin 0.5, 0.45, 0.80, vb.)
     */
    public ArmManuelCommand(Arm arm, double speed) {
        this.arm = arm;
        this.speed = speed;
        // PIDController'da belirlediğimiz tolerans değerini burada da kullanıyoruz (örneğin, 0.05)
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        // Arm alt sistemine hedef pozisyonu gönder
        arm.manualMove(speed);
    }

    @Override
    public void execute() {
        // Arm alt sistemi periodic() içinde kendi PID kontrolünü yürütüyor, burada ekstra işlem gerekmez.
    }

    @Override
    public boolean isFinished() {
        // Arm'ın mevcut pozisyonu ile hedef pozisyon arasındaki fark toleransın altındaysa komut tamamlanır.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Komut tamamlandığında veya kesildiğinde arm motorunu durdur.
        arm.stop();
    }
}
