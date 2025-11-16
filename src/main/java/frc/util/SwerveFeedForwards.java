package frc.util;

import static edu.wpi.first.units.Units.Meters;

import frc.robot.subsystems.swerve.SwerveConfig;
import java.util.function.BooleanSupplier;

public class SwerveFeedForwards {

    private SwerveFeedForwards() {}

    /**
     * Constants for linear force feedforward calculations.
     * Satisfies the equation: V_applied = kS * sign(velocityRadiansPerSecond) + kV * velocityRadiansPerSecond + kF * calculatedLinearForceFFVolts
     */
    public static record LinearForceFeedForwardConstants(double kS, double kV, double kF) {}

    /**
     * Constants for simple feedforward calculations.
     * Satisfies the equation: V_applied = kS * sign(velocityRadiansPerSecond) + kV * velocityRadiansPerSecond
     */
    public static record SimpleFeedForwardConstants(double kS, double kV) {}

    // Drive Motor Characterization Values
    public static final SimpleFeedForwardConstants simpleDriveFFConstants = new SimpleFeedForwardConstants(
        0.17388,
        0.13632
    );
    public static final SimpleFeedForwardConstants simpleDriveFFConstantsSim = new SimpleFeedForwardConstants(
        0.03197,
        0.16211
    );

    // TODO: Tune these values
    public static final LinearForceFeedForwardConstants linearForceDriveFFConstants =
        new LinearForceFeedForwardConstants(0, 0, 1);
    public static final LinearForceFeedForwardConstants linearForceDriveFFConstantsSim =
        new LinearForceFeedForwardConstants(0.0752, 0.0436, 0.8849);

    // SysId
    // public static final double driveSimKs = -0.10543;
    // public static final double driveSimKv = 0.16202;
    // public static final double driveSimKa = 0.051283;

    /**
     * Calculates the required voltage for the given linear forces and desired ground speed.
     * @param feedforwardLinearForcesNewtons - The desired linear forces in Newtons.
     * @param desiredSpeedRadPerSec - The desired ground speed in meters per second.
     * @return The required voltage to achieve the desired forces and speed.
     */
    public static double getLinearForcesFFVoltsFromRadPerSec(
        double feedforwardLinearForcesNewtons,
        double desiredSpeedRadPerSec
    ) {
        // Calculate ff voltage
        // Adapted from YAGSL SwerveDrive.drive

        // from the module configuration, obtain necessary information to calculate feed-forward
        // Warning: Will not work well if motor is not what we are expecting.

        // calculation:
        return SwerveConfig.driveGearbox.getVoltage(
            // Since: (1) torque = force * momentOfForce; (2) torque (on wheel) = torque (on motor) * gearRatio
            // torque (on motor) = force * wheelRadius / gearRatio
            (feedforwardLinearForcesNewtons * SwerveConfig.WHEEL_RADIUS.in(Meters)) / SwerveConfig.DRIVE_GEAR_RATIO,
            // Since: (1) linear velocity = angularVelocity * wheelRadius; (2) wheelVelocity = motorVelocity / gearRatio
            // motorAngularVelocity = linearVelocity / wheelRadius * gearRatio
            desiredSpeedRadPerSec * SwerveConfig.DRIVE_GEAR_RATIO
        );
    }

    /**
     * Calculates the required voltage for the given linear forces and desired ground speed.
     * @param feedforwardLinearForcesNewtons - The desired linear forces in Newtons.
     * @param desiredGroundSpeedMPS - The desired ground speed in meters per second.
     * @return The required voltage to achieve the desired forces and speed.
     */
    public static double getLinearForcesFFVoltsFromMPS(
        double feedforwardLinearForcesNewtons,
        double desiredGroundSpeedMPS
    ) {
        return getLinearForcesFFVoltsFromRadPerSec(
            feedforwardLinearForcesNewtons,
            desiredGroundSpeedMPS / SwerveConfig.WHEEL_RADIUS.in(Meters)
        );
    }

    /**
     * Calculates the required voltage for the given simple feedforward constants and desired velocity.
     * @param ffConstants - The simple feedforward constants.
     * @param velocityRadPerSec - The desired velocity in radians per second.
     * @return The required voltage to achieve the desired velocity.
     */
    public static double getSimpleFFVolts(SimpleFeedForwardConstants ffConstants, double velocityRadPerSec) {
        return ffConstants.kS * Math.signum(velocityRadPerSec) + ffConstants.kV * velocityRadPerSec;
    }

    /**
     * Calculates the required voltage for the given simple feedforward constants and desired velocity.
     * @param isSimulationSupplier - Supplier that returns true if in simulation, false otherwise.
     * @param velocityRadPerSec - The desired velocity in radians per second.
     * @return The required voltage to achieve the desired velocity.
     */
    public static double getSimpleFFVolts(BooleanSupplier isSimulationSupplier, double velocityRadPerSec) {
        SimpleFeedForwardConstants ffConstants = isSimulationSupplier.getAsBoolean()
            ? simpleDriveFFConstantsSim
            : simpleDriveFFConstants;

        return getSimpleFFVolts(ffConstants, velocityRadPerSec);
    }

    /**
     * Calculates the required voltage for the given linear force feedforward constants, desired velocity, and feedforward linear forces.
     * @param ffConstants - The linear force feedforward constants.
     * @param desiredVelocityRadPerSec - The desired velocity in radians per second.
     * @param feedforwardLinearForcesNewtons - The desired linear forces in Newtons.
     * @return The required voltage to achieve the desired velocity and forces.
     */
    public static double getLinearForceFFVolts(
        LinearForceFeedForwardConstants ffConstants,
        double desiredVelocityRadPerSec,
        double feedforwardLinearForcesNewtons
    ) {
        return (
            ffConstants.kS * Math.signum(desiredVelocityRadPerSec) +
            ffConstants.kV * desiredVelocityRadPerSec +
            ffConstants.kF *
            getLinearForcesFFVoltsFromRadPerSec(feedforwardLinearForcesNewtons, desiredVelocityRadPerSec)
        );
    }

    public static double getLinearForceFFVolts(
        BooleanSupplier isSimulationSupplier,
        double desiredVelocityRadPerSec,
        double feedforwardLinearForcesNewtons
    ) {
        LinearForceFeedForwardConstants ffConstants = isSimulationSupplier.getAsBoolean()
            ? linearForceDriveFFConstantsSim
            : linearForceDriveFFConstants;

        return getLinearForceFFVolts(ffConstants, desiredVelocityRadPerSec, feedforwardLinearForcesNewtons);
    }
}
