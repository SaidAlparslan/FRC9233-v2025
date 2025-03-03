package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;

public class ArmCommand extends Command {
    private final Arm arm;
    private final ArmState m_state;
    private final double tolerance;

    /**
     * ArmCommand, arm'ı belirli bir pozisyona getirmek için kullanılır.
     *
     * @param arm             Arm alt sistemi nesnesi
     * @param targetPosition  Hedef pozisyon (encoder birimine göre, örneğin 0.5, 0.45, 0.80, vb.)
     */
    public ArmCommand(Arm arm, ArmState state) {
        this.arm = arm;
        this.m_state = state;
        // PIDController'da belirlediğimiz tolerans değerini burada da kullanıyoruz (örneğin, 0.05)
        this.tolerance = 1;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        // Arm alt sistemine hedef pozisyonu gönder
        System.out.println("Arm: " + m_state);
        arm.setTargetState(m_state);
    }

    @Override
    public void execute() {
        // Arm alt sistemi periodic() içinde kendi PID kontrolünü yürütüyor, burada ekstra işlem gerekmez.
    }

     @Override
    public boolean isFinished() {
        
        // Arm'ın mevcut pozisyonu ile hedef pozisyon arasındaki fark toleransın altındaysa komut tamamlanır.
        if (Robot.isSimulation()) {
        return true;
    }
    // Gerçek robot için pozisyon farkı toleransın altındaysa bitir
    return Math.abs(arm.getCurrentPosition() - m_state.getPosition()) < tolerance;
    } 


    @Override
    public void end(boolean interrupted) {
        // Komut tamamlandığında veya kesildiğinde arm motorunu durdur.
        arm.stop();
    }
}
