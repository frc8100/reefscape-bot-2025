package frc.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutTemperature;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * IO interface for coupled YAMS subsystems using Spark motor controllers.
 * - A coupled YAMS subsystem is one that only uses one motor controller to control a mechanism.
 */
public interface CoupledYAMSSubsystemIO {
    /**
     * The data structure for Spark motor controllers.
     */
    public static class SparkMotorControllerData implements StructSerializable {

        private static final int NUMBER_OF_FIELDS = 5;

        /**
         * The position of the mechanism. Multiplied by a conversion factor from the motor rotations.
         */
        public final MutAngle positionAngle = Radians.mutable(0);

        /**
         * The velocity of the mechanism. Multiplied by a conversion factor from the motor RPM.
         */
        public final MutAngularVelocity velocity = RadiansPerSecond.mutable(0);

        /**
         * The applied voltage to the motor.
         * Calculated as (applied output) * (bus voltage).
         */
        public final MutVoltage appliedVolts = Volts.mutable(0);

        /**
         * The torque current applied by the motor.
         */
        public final MutCurrent torqueCurrent = Amps.mutable(0);

        /**
         * The temperature of the motor. Not simulated.
         */
        public final MutTemperature temperature = Celsius.mutable(0);

        /**
         * Creates a new Spark motor controller data object with default values (zeroes).
         */
        public SparkMotorControllerData() {}

        /**
         * Creates a new Spark motor controller data object from the given double values.
         */
        public SparkMotorControllerData(
            double positionAngleRad,
            double velocityRadPerSec,
            double appliedVolts,
            double torqueCurrentAmps,
            double temperatureCelsius
        ) {
            this.positionAngle.mut_replace(positionAngleRad, Radians);
            this.velocity.mut_replace(velocityRadPerSec, RadiansPerSecond);
            this.appliedVolts.mut_replace(appliedVolts, Volts);
            this.torqueCurrent.mut_replace(torqueCurrentAmps, Amps);
            this.temperature.mut_replace(temperatureCelsius, Celsius);
        }

        /**
         * Creates a new Spark motor controller data object from the given Measurement values.
         */
        public SparkMotorControllerData(
            Angle positionAngle,
            AngularVelocity velocity,
            Voltage appliedVolts,
            Current torqueCurrent,
            Temperature temperature
        ) {
            this.positionAngle.mut_replace(positionAngle);
            this.velocity.mut_replace(velocity);
            this.appliedVolts.mut_replace(appliedVolts);
            this.torqueCurrent.mut_replace(torqueCurrent);
            this.temperature.mut_replace(temperature);
        }

        /**
         * The struct used to serialize and deserialize Spark motor controller data.
         */
        public static final SparkMotorControllerDataStruct struct = new SparkMotorControllerDataStruct();

        /**
         * The struct implementation for Spark motor controller data.
         */
        public static class SparkMotorControllerDataStruct implements Struct<SparkMotorControllerData> {

            @Override
            public Class<SparkMotorControllerData> getTypeClass() {
                return SparkMotorControllerData.class;
            }

            @Override
            public String getTypeName() {
                return "SparkMotorControllerData";
            }

            @Override
            public int getSize() {
                return NUMBER_OF_FIELDS * kSizeDouble;
            }

            @Override
            public String getSchema() {
                return (
                    "double positionAngleRad;" +
                    "double velocityRadPerSec;" +
                    "double appliedVolts;" +
                    "double torqueCurrentAmps;" +
                    "double temperatureCelsius;"
                );
            }

            @Override
            public SparkMotorControllerData unpack(ByteBuffer bb) {
                return new SparkMotorControllerData(
                    bb.getDouble(),
                    bb.getDouble(),
                    bb.getDouble(),
                    bb.getDouble(),
                    bb.getDouble()
                );
            }

            @Override
            public void pack(ByteBuffer bb, SparkMotorControllerData data) {
                bb.putDouble(data.positionAngle.magnitude());
                bb.putDouble(data.velocity.magnitude());
                bb.putDouble(data.appliedVolts.magnitude());
                bb.putDouble(data.torqueCurrent.magnitude());
                bb.putDouble(data.temperature.magnitude());
            }
        }
    }

    /**
     * Updates the data from a Spark motor controller wrapped in a SparkWrapper.
     * @param dataToUpdate - The data to update.
     * @param motorController - The motor controller to get data from.
     * @return Whether the data was successfully updated (no sticky faults).
     */
    public static boolean updateDataFromWrappedMotorController(
        SparkMotorControllerData dataToUpdate,
        SparkWrapper motorController
    ) {
        SparkBase spark = (SparkBase) motorController.getMotorController();

        // Reset spark sticky fault
        SparkUtil.sparkStickyFault = false;

        dataToUpdate.positionAngle.mut_replace(
            SparkUtil.ifOkElseValue(spark, motorController::getMechanismPosition, Radians.zero())
        );
        dataToUpdate.velocity.mut_replace(
            SparkUtil.ifOkElseValue(spark, motorController::getMechanismVelocity, RadiansPerSecond.zero())
        );
        dataToUpdate.appliedVolts.mut_replace(
            SparkUtil.ifOkElseValue(spark, motorController::getVoltage, Volts.zero())
        );
        dataToUpdate.torqueCurrent.mut_replace(
            SparkUtil.ifOkElseValue(spark, motorController::getStatorCurrent, Amps.zero())
        );
        dataToUpdate.temperature.mut_replace(
            SparkUtil.ifOkElseValue(spark, motorController::getTemperature, Celsius.zero())
        );

        return !SparkUtil.sparkStickyFault;
    }

    /**
     * Updates the data from a Spark motor controller and its relative encoder.
     * @param dataToUpdate - The data to update.
     * @param motorController - The motor controller to get data from.
     * @param relativeEncoder - The relative encoder to get position and velocity from.
     * @return Whether the data was successfully updated (no sticky faults).
     */
    public static boolean updateDataFromSpark(
        SparkMotorControllerData dataToUpdate,
        SparkBase motorController,
        RelativeEncoder relativeEncoder
    ) {
        // Reset spark sticky fault
        SparkUtil.sparkStickyFault = false;

        dataToUpdate.positionAngle.mut_replace(
            SparkUtil.ifOkElseValue(motorController, relativeEncoder::getPosition, 0.0),
            Radians
        );
        dataToUpdate.velocity.mut_replace(
            SparkUtil.ifOkElseValue(motorController, relativeEncoder::getVelocity, 0.0),
            RadiansPerSecond
        );
        dataToUpdate.appliedVolts.mut_replace(
            SparkUtil.ifOkElseValue(
                motorController,
                () -> motorController.getAppliedOutput() * motorController.getBusVoltage(),
                0.0
            ),
            Volts
        );
        dataToUpdate.torqueCurrent.mut_replace(
            SparkUtil.ifOkElseValue(motorController, motorController::getOutputCurrent, 0.0),
            Amps
        );
        dataToUpdate.temperature.mut_replace(
            SparkUtil.ifOkElseValue(motorController, motorController::getMotorTemperature, 0.0),
            Celsius
        );

        return !SparkUtil.sparkStickyFault;
    }

    /**
     * @return The motor controller used by the subsystem.
     */
    public abstract SmartMotorController getMotorController();
}
