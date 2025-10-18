package frc.lib.util;

import org.ironmaple.simulation.SimulatedArena;

/**
 * An empty maple sim arena with no obstacles for testing purposes.
 */
public class EmptySimulationArena extends SimulatedArena {

    public EmptySimulationArena() {
        super(new FieldMap() {});
    }

    @Override
    public void placeGamePiecesOnField() {}
}
