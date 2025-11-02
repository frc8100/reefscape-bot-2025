package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

/**
 * Contains more complex characterization routines.
 */
public class SwerveSysidRoutines {

    private SwerveSysidRoutines() {}

    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    /**
     * Measures the velocity feedforward constants for the drive motors.
     * <p>This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(SwerveDrive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
            // Reset data
            Commands.runOnce(() -> {
                velocitySamples.clear();
                voltageSamples.clear();
            }),
            // Allow modules to orient
            Commands.run(() -> drive.runCharacterization(0.0), drive).withTimeout(FF_START_DELAY),
            // Start timer
            Commands.runOnce(timer::restart),
            // Accelerate and gather data
            Commands.run(
                () -> {
                    double voltage = timer.get() * FF_RAMP_RATE;
                    drive.runCharacterization(voltage);
                    velocitySamples.add(drive.getFFCharacterizationVelocity());
                    voltageSamples.add(voltage);
                },
                drive
            ).finallyDo(() -> { // When cancelled, calculate and print results
                int n = velocitySamples.size();
                double sumX = 0.0;
                double sumY = 0.0;
                double sumXY = 0.0;
                double sumX2 = 0.0;
                for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                }
                double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                NumberFormat formatter = new DecimalFormat("#0.00000");
                System.out.println("********** Drive FF Characterization Results **********");
                System.out.println("\tkS: " + formatter.format(kS));
                System.out.println("\tkV: " + formatter.format(kV));
            })
        );
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(SwerveDrive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(() -> limiter.reset(0.0)),
                // Turn in place, accelerating up to full speed
                Commands.run(
                    () -> {
                        double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                        drive.runVelocityChassisSpeeds(new ChassisSpeeds(0.0, 0.0, speed));
                    },
                    drive
                )
            ),
            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),
                // Record starting measurement
                Commands.runOnce(() -> {
                    state.positions = drive.getWheelRadiusCharacterizationPositions();
                    state.lastAngle = drive.getRotation();
                    state.gyroDelta = 0.0;
                }),
                // Update gyro delta
                Commands.run(() -> {
                    var rotation = drive.getRotation();
                    state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                    state.lastAngle = rotation;
                }).finallyDo(() -> { // When cancelled, calculate and print results
                    double[] positions = drive.getWheelRadiusCharacterizationPositions();
                    double wheelDelta = 0.0;
                    for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                    }
                    double wheelRadius = (state.gyroDelta * SwerveConfig.DRIVE_BASE_RADIUS.in(Meters)) / wheelDelta;

                    NumberFormat formatter = new DecimalFormat("#0.000");
                    System.out.println("********** Wheel Radius Characterization Results **********");
                    System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                    System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                    System.out.println(
                        "\tWheel Radius: " +
                        formatter.format(wheelRadius) +
                        " meters, " +
                        formatter.format(Units.metersToInches(wheelRadius)) +
                        " inches"
                    );
                })
            )
        );
    }

    private static class WheelRadiusCharacterizationState {

        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }

    /**
     * Drives the robot until the wheels slip. The robot should be facing a wall.
     * From https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/docs/getting-started/template-projects/spark-swerve-template.md:
     * 1. Place the robot against the solid wall.
     * 2. Using AdvantageScope, plot the current of a drive motor from the /Drive/Module.../DriveCurrentAmps key, and the velocity of the motor from the /Drive/Module.../DriveVelocityRadPerSec key.
     * 3. Accelerate forward until the drive velocity increases (the wheel slips). Note the current at this time.
     * 4. Update the value of driveMotorCurrentLimit to this value.
     */
    public static Command wheelSlipCurrentCharacterization(Swerve drive) {
        return Commands.run(
            () -> {
                drive.runCharacterization(7);
            },
            drive
        )
            .until(() -> drive.getWheelSlippingCharacterization().isPresent())
            .andThen(() -> {
                double slipData = drive.getWheelSlippingCharacterization().get();
                NumberFormat formatter = new DecimalFormat("#0.000");
                System.out.println("********** Wheel Slip Current Characterization Results **********");
                System.out.println("\tSlip Current: " + formatter.format(slipData) + " Amps");
            })
            .withTimeout(Seconds.of(10));
    }
}
