package frc.robot.subsystems.superstructure.claw;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

/**
 * The simulator implementation for the claw.
 */
public class ClawIOSim implements ClawIO {
    /**
     * The simulation model for the claw angle motor.
     */
    private final DCMotor angleMotorGearbox = ClawConstants.SIM_ANGLE_MOTOR;

    /**
     * The simulated angle motor. Controls the angle of the claw.
     */
    private final DCMotorSim angleMotorSim = new DCMotorSim(
            LinearSystemId.createSingleJointedArmSystem(
                    angleMotorGearbox, ClawConstants.SIM_ANGLE_MOI, ClawConstants.ANGLE_GEAR_RATIO),
            angleMotorGearbox);

    /**
     * The PID angle controller for the claw.
     */
    private final PIDController angleController =
            new PIDController(ClawConstants.ANGLE_KP, ClawConstants.ANGLE_KI, ClawConstants.ANGLE_KD);

    /**
     * The simulation model for the claw outtake motor.
     */
    private final DCMotor outtakeMotorGearbox = ClawConstants.SIM_OUTTAKE_MOTOR;

    /**
     * The simulated outtake motor. Controls the outtake of the claw.
     */
    private final DCMotorSim outtakeMotorSim = new DCMotorSim(
            LinearSystemId.createSingleJointedArmSystem(
                    outtakeMotorGearbox, ClawConstants.SIM_OUTTAKE_MOI, ClawConstants.OUTTAKE_GEAR_RATIO),
            outtakeMotorGearbox);

    @Override
    public void stop() {
        // Stop the angle motor
        // TODO: Also stop PID controller
        angleMotorSim.setInputVoltage(0);
        outtakeMotorSim.setInputVoltage(0);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        // Set the setpoint of the angle controller
        angleController.setSetpoint(rotation.getRadians());
    }

    @Override
    public void runOutake(double motorInput) {
        // Set the output of the outtake motor
        // TODO: voltage
        // outtakeMotorSim.setInputVoltage(motorInput * ClawConstants.MAX_OUTTAKE_POWER * 12);
        outtakeMotorSim.setAngularVelocity(motorInput * ClawConstants.MAX_OUTTAKE_POWER * 200);
    }

    @Override
    public void periodic() {
        // Set the output of the motors based on the PID controller
        double angleMotorOutput = angleController.calculate(angleMotorSim.getAngularPositionRad());

        angleMotorSim.setInputVoltage(angleMotorOutput);

        // Debug
        Logger.recordOutput("ClawSim/angleMotorOutput", angleMotorOutput);

        // Update the simulation
        // TODO: add dt
        angleMotorSim.update(0.02);
        outtakeMotorSim.update(0.02);
    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        // Set angle inputs
        inputs.turnConnected = true;
        inputs.turnPositionRad = angleMotorSim.getAngularPositionRad();
        inputs.turnVelocityRadPerSec = angleMotorSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts = angleMotorSim.getInputVoltage();
        inputs.turnSupplyCurrentAmps = angleMotorSim.getCurrentDrawAmps();
        inputs.turnTorqueCurrentAmps = angleMotorSim.getCurrentDrawAmps();
        inputs.turnTempCelsius = 0;

        // Set outtake inputs
        inputs.outakeConnected = true;
        inputs.outakePositionRad = outtakeMotorSim.getAngularPositionRad();
        inputs.outakeVelocityRadPerSec = outtakeMotorSim.getAngularVelocityRadPerSec();
        inputs.outakeAppliedVolts = outtakeMotorSim.getInputVoltage();
        inputs.outakeSupplyCurrentAmps = outtakeMotorSim.getCurrentDrawAmps();
        inputs.outakeTorqueCurrentAmps = outtakeMotorSim.getCurrentDrawAmps();
        inputs.outakeTempCelsius = 0;
    }
}
