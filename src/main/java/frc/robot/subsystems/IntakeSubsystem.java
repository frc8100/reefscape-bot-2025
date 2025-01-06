package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax m_motor;

    private double m_power = -1;
    private double m_epower = .50;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        // create a new SPARK MAX and configure it
        m_motor = new CANSparkMax(Constants.Intake.kCanId, MotorType.kBrushed);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.burnFlash();
    }

    public Command intake() {
        return startEnd(
            () -> {
                m_motor.set(m_power);
            },
            () -> {
                m_motor.set(0);
            }
        );
    }

    public Command eintake() {
        return startEnd(
            () -> {
                m_motor.set(m_epower);
            },
            () -> {
                m_motor.set(0);
            }
        );
    }
}
