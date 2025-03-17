package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private final DigitalOutput redLED;
    private final DigitalOutput greenLED;
    private final DigitalOutput blueLED;
    private final Timer blinkTimer = new Timer();
    private boolean isBlinking = false;

    public LED(int redPort, int greenPort, int bluePort) {
        redLED = new DigitalOutput(redPort);
        greenLED = new DigitalOutput(greenPort);
        blueLED = new DigitalOutput(bluePort);
        blinkTimer.start();
    }

    public void setColor(boolean red, boolean green, boolean blue) {
        redLED.set(red);
        greenLED.set(green);
        blueLED.set(blue);
        SmartDashboard.putBoolean("LED-RED: ", red);
        SmartDashboard.putBoolean("LED-GREEN: ", green);
        SmartDashboard.putBoolean("LED-BLUE: ", blue);
    }

    public void setRed() {
        isBlinking = false;
        setColor(true, false, false);
    }

    public void setGreen() {
        isBlinking = false;
        setColor(false, true, false);

    }

    public void blinkRed() {
        isBlinking = true;
    }

    public void turnOff() {
        isBlinking = false;
        setColor(false, false, false);
    }

    @Override
    public void periodic() {
        if (isBlinking) {
            boolean blinkState = ((int) (blinkTimer.get() * 2) % 2 == 0);
            setColor(blinkState, false, false);
        }
    }
}
