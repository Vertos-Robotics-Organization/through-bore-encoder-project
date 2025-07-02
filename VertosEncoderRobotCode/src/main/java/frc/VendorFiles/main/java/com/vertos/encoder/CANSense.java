package frc.VendorFiles.main.java.com.vertos.encoder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class CANSense extends ParentCANSense implements Sendable, AutoCloseable {

    public CANSense(int canId, boolean isDebug) {
        super(canId, isDebug);
    }

    @Override
    public void close() throws Exception {
        SendableRegistry.remove(this);

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("CANSense");
        builder.addDoubleProperty("Position", () -> getAbsRotations(), this::setPosition);
    }
    
}
