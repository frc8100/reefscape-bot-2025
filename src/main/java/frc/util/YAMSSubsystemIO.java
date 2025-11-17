package frc.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.filter.Debouncer;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SparkWrapper;

public interface YAMSSubsystemIO {
    /**
     * What is record in the IO inputs for a Spark motor controller.
     * @param motorConnected - Whether the motor is connected.
     * @param positionRad - The position of the motor in radians.
     * @param velocityRadPerSec - The velocity of the motor in radians per second.
     * @param appliedVolts - The applied voltage to the motor.
     * @param torqueCurrentAmps - The torque current of the motor in amps.
     * @param tempCelsius - The temperature of the motor in celsius.
     */
    public static record SparkMotorControllerData(
        double positionRad,
        double velocityRadPerSec,
        double appliedVolts,
        double torqueCurrentAmps,
        double tempCelsius,
        boolean motorConnected
    ) {}

    public static SparkMotorControllerData defaultControllerData = new SparkMotorControllerData(
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        true
    );

    /**
     * Gets the data from a Spark motor controller.
     * @param motorController - The motor controller to get data from.
     * @param motorConnectionDebouncer - The debouncer for the motor connection status.
     * @return The data from the motor controller.
     */
    public static SparkMotorControllerData getDataFromMotorController(
        SparkWrapper motorController,
        Debouncer motorConnectionDebouncer
    ) {
        SparkBase spark = (SparkBase) motorController.getMotorController();

        // Reset spark sticky fault
        SparkUtil.sparkStickyFault = false;

        return new SparkMotorControllerData(
            SparkUtil.ifOkOtherwiseZero(spark, () -> motorController.getMechanismPosition().in(Radians)),
            SparkUtil.ifOkOtherwiseZero(spark, () -> motorController.getMechanismVelocity().in(RadiansPerSecond)),
            SparkUtil.ifOkOtherwiseZero(spark, () -> motorController.getVoltage().in(Volts)),
            SparkUtil.ifOkOtherwiseZero(spark, () -> motorController.getStatorCurrent().in(Amps)),
            SparkUtil.ifOkOtherwiseZero(spark, () -> motorController.getTemperature().in(Celsius)),
            motorConnectionDebouncer.calculate(!SparkUtil.sparkStickyFault)
        );
    }

    public abstract SmartMotorController getMotorController();
}
