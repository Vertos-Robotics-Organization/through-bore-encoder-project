package frc.VendorFiles.main.java.com.vertos.encoder;

import java.util.BitSet;

public class CoreDeviceFaultLayer {

    // Fault status variables (kept for backward compatibility)
    private boolean isStickyFault_Hardware = false; 
    private boolean isStickyFault_BootDuringEnable = false;
    private boolean isStickyFault_LoopOverrun = false;
    private boolean isStickyFault_BadMagnet = false;
    private boolean isStickyFault_CANGeneral = false;
    private boolean isStickyFault_MomentaryCanBusLoss = false; 
    private boolean isStickyFault_CANClogged = false;
    private boolean isStickyFault_RotationOverspeed = false;
    private boolean isStickyFault_UnderVolted = false;

    // Non-sticky fault status variables
    private boolean isFault_Hardware = false; 
    private boolean isFault_BootDuringEnable = false;
    private boolean isFault_LoopOverrun = false;
    private boolean isFault_BadMagnet = false;
    private boolean isFault_CANGeneral = false;
    private boolean isFault_MomentaryCanBusLoss = false; 
    private boolean isFault_CANClogged = false;
    private boolean isFault_RotationOverspeed = false;
    private boolean isFault_UnderVolted = false;

    private final int NUM_FAULTS = 8; // Number of fault bits (0-7)
    private final BitSet faults = new BitSet(NUM_FAULTS);
    private final BitSet stickyFaults = new BitSet(NUM_FAULTS);



    public CoreDeviceFaultLayer() {

    }


    //----------------------------------------------------------------------------------------
    // Fault Status Methods
    //----------------------------------------------------------------------------------------
    /**
     * Retrieves the sticky hardware fault status.
     * (Updated to use boolean status data)
     *
     * @return True if a sticky hardware fault has been detected, false otherwise.
     */
    public boolean getStickyFault_Hardware() {
        return isStickyFault_Hardware;
    }

    /**
     * Resets the sticky hardware fault status.
     */
    public void resetStickyFault_Hardware() {
        isStickyFault_Hardware = false;
    }

    /**
     * Retrieves the sticky boot during enable fault status.
     *
     * @return True if a sticky boot during enable fault has been detected, false otherwise.
     */
    public boolean getStickyFault_BootDuringEnable() {
        return isStickyFault_BootDuringEnable;
    }

    /**
     * Resets the sticky boot during enable fault status.
     */
    public void resetStickyFault_BootDuringEnable() {
        isStickyFault_BootDuringEnable = false;
    }

    /**
     * Retrieves the sticky magnet fault status.
     *
     * @return True if a sticky magnet fault has been detected, false otherwise.
     */
    public boolean getStickyFault_BadMagnet() {
        return isStickyFault_BadMagnet;
    }

    /**
     * Resets the sticky magnet fault status.
     */
    public void resetStickyFault_BadMagnet() {
        isStickyFault_BadMagnet = false;
    }

    /**
     * Retrieves the sticky general CAN fault status.
     * (Updated to use boolean status data)
     *
     * @return True if a sticky general CAN fault has been detected, false otherwise.
     */
    public boolean getStickyFault_CANGeneral() {
        return isStickyFault_CANGeneral;
    }

    /**
     * Resets the sticky general CAN fault status.
     */
    public void resetStickyFault_CANGeneral() {
        isStickyFault_CANGeneral = false;
    }

    /**
     * Retrieves the sticky loop overrun fault status.
     *
     * @return True if a sticky loop overrun fault has been detected, false otherwise.
     */
    public boolean getStickyFault_LoopOverrun() {
        return isStickyFault_LoopOverrun;
    }

    /**
     * Resets the sticky loop overrun fault status.
     */
    public void resetStickyFault_LoopOverrun() {
        isStickyFault_LoopOverrun = false;
    }

    /**
     * Retrieves the sticky momentary CAN bus loss fault status.
     *
     * @return True if a sticky momentary CAN bus loss fault has been detected, false otherwise.
     */
    public boolean getStickyFault_MomentaryCanBusLoss() {
        return isStickyFault_MomentaryCanBusLoss;
    }

    /**
     * Resets the sticky momentary CAN bus loss fault status.
     */
    public void resetStickyFault_MomentaryCanBusLoss() {
        isStickyFault_MomentaryCanBusLoss = false;
    }

    /**
     * Retrieves the sticky CAN clogged fault status.
     *
     * @return True if a sticky CAN clogged fault has been detected, false otherwise.
     */
    public boolean getStickyFault_CANClogged() {
        return isStickyFault_CANClogged;
    }

    /**
     * Resets the sticky CAN clogged fault status.
     */
    public void resetStickyFault_CANClogged() {
        isStickyFault_CANClogged = false;
    }

    /**
     * Retrieves the sticky rotation overspeed fault status.
     *
     * @return True if a sticky rotation overspeed fault has been detected, false otherwise.
     */
    public boolean getStickyFault_RotationOverspeed() {
        return isStickyFault_RotationOverspeed;
    }

    /**
     * Resets the sticky rotation overspeed fault status.
     */
    public void resetStickyFault_RotationOverspeed() {
        isStickyFault_RotationOverspeed = false;
    }

    /**
     * Retrieves the sticky under-volted fault status.
     *
     * @return True if a sticky under-volted fault has been detected, false otherwise.
     */
    public boolean getStickyFault_UnderVolted() {
        return isStickyFault_UnderVolted;
    }

    /**
     * Resets the sticky under-volted fault status.
     */
    public void resetStickyFault_UnderVolted() {
        isStickyFault_UnderVolted = false;
    }

    /**
     * Retrieves the hardware fault status.
     *
     * @return True if a hardware fault has been detected, false otherwise.
     */
    public boolean Error_Hardware() {
        return isFault_Hardware;
    }

    /**
     * Retrieves the boot during enable fault status.
     *
     * @return True if a boot during enable fault has been detected, false otherwise.
     */
    public boolean Error_BootDuringEnable() {
        return isFault_BootDuringEnable;
    }

    /**
     * Retrieves the loop overrun fault status.
     *
     * @return True if a loop overrun fault has been detected, false otherwise.
     */
    public boolean Warning_LoopOverrun() {
        return isFault_LoopOverrun;
    }

    /**
     * Retrieves the bad magnet fault status.
     *
     * @return True if a bad magnet fault has been detected, false otherwise.
     */
    public boolean Error_BadMagnet() {
        return isFault_BadMagnet;
    }

    /**
     * Retrieves the general CAN fault status.
     *
     * @return True if a general CAN fault has been detected, false otherwise.
     */
    public boolean Warning_CANGeneral() {
        return isFault_CANGeneral;
    }

    /**
     * Retrieves the momentary CAN bus loss fault status.
     *
     * @return True if a momentary CAN bus loss fault has been detected, false otherwise.
     */
    public boolean Warning_MomentaryCanBusLoss() {
        return isFault_MomentaryCanBusLoss;
    }

    /**
     * Retrieves the CAN clogged fault status.
     *
     * @return True if a CAN clogged fault has been detected, false otherwise.
     */
    public boolean Warning_CANClogged() {
        return isFault_CANClogged;
    }

    /**
     * Retrieves the rotation overspeed fault status.
     *
     * @return True if a rotation overspeed fault has been detected, false otherwise.
     */
    public boolean Error_RotationOverspeed() {
        return isFault_RotationOverspeed;
    }

    /**
     * Retrieves the under-volted fault status.
     *
     * @return True if an under-volted fault has been detected, false otherwise.
     */
    public boolean Error_UnderVolted() {
        return isFault_UnderVolted;
    }

    /**
     * Resets all sticky fault flags to false.
     * This method allows the user to clear all sticky fault statuses.
     */
    public void resetStickyFaults() {
        // Clear local fault states
        isStickyFault_Hardware = false;
        isStickyFault_CANGeneral = false;
        isStickyFault_LoopOverrun = false;
        isStickyFault_MomentaryCanBusLoss = false;
        isStickyFault_BootDuringEnable = false;
        isStickyFault_BadMagnet = false;
        isStickyFault_CANClogged = false;
        isStickyFault_RotationOverspeed = false;
        isStickyFault_UnderVolted = false;
    }

    public CoreDeviceListener getFaultListener() {
        return faultListener;
    }

    private boolean previousCANFault = false;

    private final CoreDeviceListener faultListener = new CoreDeviceListener() {
        @Override
        public void onDataReceived(byte[] data) {
            synchronized (CoreDeviceFaultLayer.this) {
                byte booleanStatusByte = data[0];

                // Extract faults using BitSet
                for (int i = 0; i < NUM_FAULTS; i++) {
                    boolean fault = (booleanStatusByte & (1 << i)) != 0;
                    faults.set(i, fault);
                    stickyFaults.set(i, stickyFaults.get(i) || fault); // Update sticky faults
                }

                // Update individual fault variables for backward compatibility
                isFault_Hardware = faults.get(0);
                isFault_LoopOverrun = faults.get(1);
                isFault_CANGeneral = faults.get(2);
                isFault_BootDuringEnable = faults.get(3);
                isFault_BadMagnet = faults.get(4);
                isFault_RotationOverspeed = faults.get(5);
                isFault_CANClogged = faults.get(6);
                isFault_UnderVolted = faults.get(7);

                isStickyFault_Hardware |= isFault_Hardware;
                isStickyFault_LoopOverrun |= isFault_LoopOverrun;
                isStickyFault_CANGeneral |= isFault_CANGeneral;
                isStickyFault_BootDuringEnable |= isFault_BootDuringEnable;
                isStickyFault_BadMagnet |= isFault_BadMagnet;
                isStickyFault_RotationOverspeed |= isFault_RotationOverspeed;
                isStickyFault_CANClogged |= isFault_CANClogged;
                isStickyFault_UnderVolted |= isFault_UnderVolted;

                // Handle CAN fault logic
                boolean currentCANFault = isFault_CANClogged || isFault_CANGeneral;
                if (previousCANFault && !currentCANFault) {
                    isStickyFault_MomentaryCanBusLoss = true;
                }
                previousCANFault = currentCANFault;
            }
        }
    };
}
