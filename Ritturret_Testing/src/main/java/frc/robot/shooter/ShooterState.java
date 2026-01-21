package frc.robot.shooter;

import java.util.function.Supplier;

/* "Inspired" by 254 2024
 * 
 * SOTM Phyzooks Inside
 * 
 */
public class ShooterState {
    private double turretPosition;
    private double turretFF;

    public ShooterState(double turretPosition, double turretFF){
        this.turretPosition = turretPosition;
        this.turretFF = turretFF;
    }

    private static ShooterState makeSetpoint(){
        return new ShooterState(0.0, 0.0);
    }

    public static ShooterState fromFerryTarget(){
        return makeSetpoint();
    }

    public static Supplier<ShooterState> getFerrySetpointSupplier(){
        return () -> fromFerryTarget();
    }

    public static ShooterState fromHub(){
        return makeSetpoint();
    }

    public static Supplier<ShooterState> getHubSetpointSupplier(){
        return () -> fromHub();
    }
}
