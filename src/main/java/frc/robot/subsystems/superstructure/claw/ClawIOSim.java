package frc.robot.subsystems.superstructure.claw;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
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
            angleMotorGearbox,
            ClawConstants.SIM_ANGLE_MOI.in(KilogramSquareMeters),
            ClawConstants.ANGLE_GEAR_RATIO
        ),
        angleMotorGearbox
    );

    /**
     * The PID angle controller for the claw.
     */
    private final PIDController angleController = new PIDController(
        ClawConstants.SIM_ANGLE_KP,
        ClawConstants.SIM_ANGLE_KI,
        ClawConstants.SIM_ANGLE_KD
    );

    /**
     * Whether the angle controller is using PID or not.
     * If false, the angle motor will not be updated by the PID controller.
     * The PID controller will still be updated.
     */
    private boolean isAngleUsingPID = true;

    /**
     * The simulation model for the claw outtake motor.
     */
    private final DCMotor outtakeMotorGearbox = ClawConstants.SIM_OUTTAKE_MOTOR;

    /**
     * The simulated outtake motor. Controls the outtake of the claw.
     */
    private final DCMotorSim outtakeMotorSim = new DCMotorSim(
        LinearSystemId.createSingleJointedArmSystem(
            outtakeMotorGearbox,
            ClawConstants.SIM_OUTTAKE_MOI.in(KilogramSquareMeters),
            ClawConstants.OUTTAKE_GEAR_RATIO
        ),
        outtakeMotorGearbox
    );

    /**
     * The PID outtake controller for the claw.
     */
    private final PIDController outtakeController = new PIDController(
        ClawConstants.SIM_OUTTAKE_KP,
        ClawConstants.SIM_OUTTAKE_KI,
        ClawConstants.SIM_OUTTAKE_KD
    );

    /**
     * Whether the outtake controller is using PID or not.
     * If false, the outtake motor will not be updated by the PID controller.
     * The PID controller will still be updated.
     */
    private boolean isOuttakeUsingPID = true;

    @Override
    public void stop() {
        // Stop the angle motor
        isAngleUsingPID = false;
        isOuttakeUsingPID = false;

        angleMotorSim.setInputVoltage(0);
        outtakeMotorSim.setInputVoltage(0);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        isAngleUsingPID = true;

        // Set the setpoint of the angle controller
        angleController.setSetpoint(rotation.getRadians());
    }

    @Override
    public void runOutake(double motorInput) {
        isOuttakeUsingPID = true;

        // Set the setpoint of the outtake controller
        outtakeController.setSetpoint(motorInput * ClawConstants.SIM_OUTTAKE_TARGET_VELOCITY.in(RadiansPerSecond));
    }

    @Override
    public void periodic() {
        // Set the output of the motors based on the PID controller
        double angleMotorOutput = angleController.calculate(angleMotorSim.getAngularPositionRad());

        if (isAngleUsingPID) {
            Logger.recordOutput("ClawSim/AnglePIDOutput", angleMotorOutput);
            angleMotorSim.setInputVoltage(MathUtil.clamp(angleMotorOutput, -12, 12));
        }

        double outtakeMotorOutput = outtakeController.calculate(outtakeMotorSim.getAngularVelocityRadPerSec());

        if (isOuttakeUsingPID) {
            Logger.recordOutput("ClawSim/OuttakePIDOutput", outtakeMotorOutput);
            outtakeMotorSim.setInputVoltage(MathUtil.clamp(outtakeMotorOutput, -12, 12));
        }

        // Update the simulation
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
        inputs.turnSetpointRad = angleController.getSetpoint();

        // Set outtake inputs
        inputs.outakeConnected = true;
        inputs.outakePositionRad = outtakeMotorSim.getAngularPositionRad();
        inputs.outakeVelocityRadPerSec = outtakeMotorSim.getAngularVelocityRadPerSec();
        inputs.outakeAppliedVolts = outtakeMotorSim.getInputVoltage();
        inputs.outakeSupplyCurrentAmps = outtakeMotorSim.getCurrentDrawAmps();
        inputs.outakeTorqueCurrentAmps = outtakeMotorSim.getCurrentDrawAmps();
        inputs.outakeSetpointVelocityRadPerSec = outtakeController.getSetpoint();
    }
}
