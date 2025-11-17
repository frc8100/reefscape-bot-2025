package frc.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SparkWrapper;

public interface YAMSSubsystemIO {
    /**
     * What is recorded in the IO inputs for a Spark motor controller.
     * @param motorConnected - Whether the motor is connected.
     * @param positionAngle - The position of the mechanism in radians. Multiplied by a conversion factor from the motor rotations.
     * @param velocity - The velocity of the motor in radians per second. Multiplied by a conversion factor from the motor rotations.
     * @param appliedVolts - The applied voltage to the motor.
     * @param torqueCurrent - The torque current of the motor in amps.
     * @param temperature - The temperature of the motor in celsius.
     */
    public static record SparkMotorControllerData(
        Angle positionAngle,
        AngularVelocity velocity,
        Voltage appliedVolts,
        Current torqueCurrent,
        Temperature temperature,
        boolean motorConnected
    ) {}

    /**
     * The default data for a Spark motor controller.
     * All values are `0` and motor connected is `true`.
     */
    public static SparkMotorControllerData defaultControllerData = new SparkMotorControllerData(
        Radians.of(0),
        RadiansPerSecond.of(0),
        Volts.of(0),
        Amps.of(0),
        Celsius.of(0),
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
            SparkUtil.ifOkElse(spark, motorController::getMechanismPosition, () -> Radians.of(0)),
            SparkUtil.ifOkElse(spark, motorController::getMechanismVelocity, () -> RadiansPerSecond.of(0)),
            SparkUtil.ifOkElse(spark, motorController::getVoltage, () -> Volts.of(0)),
            SparkUtil.ifOkElse(spark, motorController::getStatorCurrent, () -> Amps.of(0)),
            SparkUtil.ifOkElse(spark, motorController::getTemperature, () -> Celsius.of(0)),
            motorConnectionDebouncer.calculate(!SparkUtil.sparkStickyFault)
        );
    }

    public abstract SmartMotorController getMotorController();
}
