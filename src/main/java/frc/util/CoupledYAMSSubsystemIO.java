package frc.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * IO interface for coupled YAMS subsystems using Spark motor controllers.
 * - A coupled YAMS subsystem is one that only uses one motor controller to control a mechanism.
 */
public interface CoupledYAMSSubsystemIO {
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
        Radians.zero(),
        RadiansPerSecond.zero(),
        Volts.zero(),
        Amps.zero(),
        Celsius.zero(),
        true
    );

    public static record SparkMotorControllerDataUnitless(
        double positionAngleRad,
        double velocityRadPerSec,
        double appliedVolts,
        double torqueCurrentAmps,
        double temperatureCelsius,
        boolean motorConnected
    ) {}

    public static SparkMotorControllerDataUnitless defaultControllerDataUnitless = new SparkMotorControllerDataUnitless(
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
    public static SparkMotorControllerData getDataFromMotorController(SparkWrapper motorController) {
        SparkBase spark = (SparkBase) motorController.getMotorController();

        // Reset spark sticky fault
        SparkUtil.sparkStickyFault = false;

        return new SparkMotorControllerData(
            SparkUtil.ifOkElseValue(spark, motorController::getMechanismPosition, Radians.zero()),
            SparkUtil.ifOkElseValue(spark, motorController::getMechanismVelocity, RadiansPerSecond.zero()),
            SparkUtil.ifOkElseValue(spark, motorController::getVoltage, Volts.zero()),
            SparkUtil.ifOkElseValue(spark, motorController::getStatorCurrent, Amps.zero()),
            SparkUtil.ifOkElseValue(spark, motorController::getTemperature, Celsius.zero()),
            !SparkUtil.sparkStickyFault
        );
    }

    /**
     * Gets the data from a Spark motor controller in unitless form.
     * @param motorController - The motor controller to get data from.
     * @param relativeEncoder - The relative encoder to get position and velocity from.
     * @return The data from the motor controller in unitless form.
     */
    public static SparkMotorControllerDataUnitless getDataFromSparkUnitless(
        SparkBase motorController,
        RelativeEncoder relativeEncoder
    ) {
        // Reset spark sticky fault
        SparkUtil.sparkStickyFault = false;

        return new SparkMotorControllerDataUnitless(
            SparkUtil.ifOkElseValue(motorController, relativeEncoder::getPosition, 0.0),
            SparkUtil.ifOkElseValue(motorController, relativeEncoder::getVelocity, 0.0),
            SparkUtil.ifOkElseValue(
                motorController,
                () -> motorController.getAppliedOutput() * motorController.getBusVoltage(),
                0.0
            ),
            SparkUtil.ifOkElseValue(motorController, motorController::getOutputCurrent, 0.0),
            SparkUtil.ifOkElseValue(motorController, motorController::getMotorTemperature, 0.0),
            !SparkUtil.sparkStickyFault
        );
    }

    /**
     * @return The motor controller used by the subsystem.
     */
    public abstract SmartMotorController getMotorController();
}
