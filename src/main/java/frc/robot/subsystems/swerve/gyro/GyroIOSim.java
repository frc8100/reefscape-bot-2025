package frc.robot.subsystems.swerve.gyro;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SparkUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

/**
 * IO implementation for the gyro simulation.
 */
public class GyroIOSim implements GyroIO {

    /**
     * Use the maplesim gyro simulation. This contains the gyro readings and angular velocity.
     */
    private final GyroSimulation gyroSimulation;

    private Rotation2d yawOffset = new Rotation2d();

    /**
     * Creates a new GyroIOSim given the gyro simulation to use.
     */
    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = gyroSimulation.getGyroReading();
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(
            gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond)
        );

        inputs.odometryYawTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
        inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
    }

    @Override
    public Rotation2d getGyroHeading() {
        return gyroSimulation.getGyroReading();
    }

    @Override
    public Rotation2d getGyroHeadingForFieldRelative() {
        return getGyroHeading().minus(yawOffset);
    }

    @Override
    public void zeroGyro(double deg) {
        gyroSimulation.setRotation(Rotation2d.fromDegrees(deg));
    }

    @Override
    public void zeroFieldRelativeGyro(double deg) {
        yawOffset = getGyroHeading().minus(Rotation2d.fromDegrees(deg));
    }
}
