package bme280pi;

import jdk.dio.DeviceManager;
import jdk.dio.i2cbus.I2CDeviceConfig;

public class Bme280Sensor {
    /** 
     * Size of Data in Register: THREE BYTES
     */
    private final int THREE_BYTES = 3;

    private Bme280i2cReadWrite bme280IO;
    private Bme280CalibrationParams bme280Calibration;

    /**
     * Create BME 280 Sensor
     * @param i2cSensorAddr - I2C BME 280 addr. Get addr by command: sudo i2cdetect -y 1
     */
    public Bme280Sensor(int i2cSensorAddr) {
        /** 
         * controller number is 1 since: sudo i2cdetect -y 1
         */
        final int CTRL_NUM = 1;

        I2CDeviceConfig bme280DeviceConfig = new I2CDeviceConfig.Builder()
            .setControllerNumber(CTRL_NUM)
            .setAddress(i2cSensorAddr, I2CDeviceConfig.ADDR_SIZE_7)
            .setClockFrequency(I2CDeviceConfig.UNASSIGNED) // bme280 has no clock inside
            .build();

        try {
            
            bme280IO = new Bme280i2cReadWrite(DeviceManager.open(bme280DeviceConfig));
        } catch (Exception e) {
            e.printStackTrace();
            bme280IO = null;
            java.lang.System.exit(1);
        }
        // initialize calibration parameters
        bme280Calibration = new Bme280CalibrationParams(this);
    }

    /**
     * Read Calibration Data.
     * @param calibReg - calibration register address
     * @return calibration data
     */
    public int readCalibrationData(int calibReg) {
        return (int) bme280IO.readRegister(calibReg);
    }

    /**
     * Return chip Id.
     * @return chip id
     */
    public long getChipId() {
        return bme280IO.readRegister(Bme280Registers.CHIP_ID_REG);
    }

    /**
     * Return chip mode.
     * 0b00 - Sleep Mode
     * 0b10 and 0b01 - Force Mode
     * 0x11 - Normal Mode
     * @return chip mode
     */
    public long getChipMode() {
        return bme280IO.readRegister(Bme280Registers.CTRL_MES_REG) & 0x03;
    }

    /**
     * Get oversampling.
     *   000 - Skipped (output set to 0x8000)
     *   001 - oversampling x 1
     *   010 - oversampling x 2
     *   011 - oversampling x 4
     *   100 - oversampling x 8
     *   101, other  - oversampling x 16
     * @param sensorType - temperature(0), pressure(1) or humidity(2)
     * @return oversampling
     */
    public long getOversampling(int sensorType) {
        if (sensorType == 0) {
            final long rawData = bme280IO.readRegister(Bme280Registers.CTRL_MES_REG);
            return (rawData & 0b11100000) >> 5;
        }
        if (sensorType == 1) {
            final long rawData = bme280IO.readRegister(Bme280Registers.CTRL_MES_REG);
            return (rawData & 0b00011100) >> 2;
        }
        if (sensorType == 2) {
            return bme280IO.readRegister(Bme280Registers.CTRL_HUM_REG) & 0b00000111;
        }
        return 0;
    }

    public void setOversampling(int sensorType, long oversampling) {
        assert sensorType >= 0 && sensorType <= 2;
        assert oversampling >=0b000 && oversampling <= 0b111;
        long regData = 0;
        if (sensorType == 0) {
            // read existing register data to overwrite only related oversampling
            regData =  bme280IO.readRegister(Bme280Registers.CTRL_MES_REG);
            regData = (regData & 0b00011111) | (oversampling << 5);
            bme280IO.writeRegister(Bme280Registers.CTRL_MES_REG, regData);
        }
        if (sensorType == 1) {
            regData =  bme280IO.readRegister(Bme280Registers.CTRL_MES_REG);
            regData = (regData & 0b11100011) | (oversampling << 2);
            bme280IO.writeRegister(Bme280Registers.CTRL_MES_REG, oversampling);
        }
        if (sensorType == 2) {
            bme280IO.writeRegister(Bme280Registers.CTRL_HUM_REG, oversampling);
        }
    }

    /**
     * Set Chip Mode.
     *   0b00 - Sleep Mode
     *   0x10 or 0x01 - Forced Mode
     *   0x11 - Normal Mode
     */
    public void setChipMode(long newMode) {
        long regData = bme280IO.readRegister(Bme280Registers.CTRL_MES_REG);
        // clear first 2 bits
        regData = regData & 0xFC;
        // set new Mode
        regData = regData | (newMode & 0x03);
        bme280IO.writeRegister(Bme280Registers.CTRL_MES_REG, regData);
    }

    /**
     * Return sensor status.
     * bit 0 - im_update
     * bit 2 - measuring
     * @return status
     */
    public long getStatus() {
        final long rawData = bme280IO.readRegister(Bme280Registers.STATUS_REG);
        return (rawData & 0x04 >> 1) | (rawData & 0x01);
    }

    /**
     * Returns temperature in DegC, resolution is 0.01 DegC. 
     * Output value of “5123” equals 51.23 DegC.
     * @return current Temperature
     */
    public long getTemperature() {
        // final int TEMP_MIN = -4000; // from data sheet 
        // final int TEMP_MAX = 8500; // from data sheet 
        long rawData = bme280IO.readRegister(Bme280Registers.TEMPERATURE_REG, THREE_BYTES);
        // Temperature Data:
        //   byte 0: Temp MSB
        //   byte 1: Temp LSB
        //   byte 2: Temp XLSB [7:4]
        // Right order is: XLSB (4 bits) then LSB (8 bits) and then MSB (8 bits)
        final long msb = (rawData & 0xFF);
        // move on 4 bits since xlsb contains only 4 bits (8-4 = 4). Actaully it should be >> 8 << 4 
        final long lsb = (rawData & 0xFF00) >> 8;
        // move on 20 bits since xlsb contains only 4 bits and it is located on 3d byte (16+4=20)
        final long xlsb = (rawData & 0xFF0000) >> 20;
        rawData = (msb << 12) | (lsb<<4) | xlsb;

        long var1 = (((rawData >> 3)  - (bme280Calibration.DigT1() << 1))*bme280Calibration.DigT2() >> 11);
        long var2 = ((rawData >> 4) - bme280Calibration.DigT1());
        var2  = ((((var2*var2) >> 12)*bme280Calibration.DigT3()) >> 14);
        bme280Calibration.t_fine = var1+var2;
        
        return (bme280Calibration.t_fine*5+128)>>8;
    }

    /**
     * Returns pressure in Pa as unsigned 32 bit integer.
     * Output value of "102354" equals 102354 Pa = 1023.54 hPa.
     * @return current Pressure
     */
    public long getPressure() {
        long rawData = bme280IO.readRegister(Bme280Registers.PRESSURE_REG, THREE_BYTES);
        // Pressure Data:
        //   byte 0: Pressure MSB
        //   byte 1: Pressure LSB
        //   byte 2: Pressure XLSB [7:4]
        // Right order is: XLSB (4 bits) then LSB (8 bits) and then MSB (8 bits)
        final long msb = (rawData & 0xFF);
        // move on 4 bits since xlsb contains only 4 bits (8-4 = 4). Actaully it should be >> 8 << 4 
        final long lsb = (rawData & 0xFF00) >> 8;
        // move on 20 bits since xlsb contains only 4 bits and it is located on 3d byte (16+4=20)
        final long xlsb = (rawData & 0xFF0000) >> 20;
        rawData = (msb << 12) | (lsb<<4) | xlsb;
        
        long var1 = (bme280Calibration.t_fine >> 1) - 64000;
        long var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * bme280Calibration.DigP6();
        var2 = var2 + ((var1 * bme280Calibration.DigP5()) << 1);
        var2 = (var2 >> 2) + (bme280Calibration.DigP4() << 16);
        var1 = (((bme280Calibration.DigP3() * (((var1 >> 2)*(var1 >> 2)) >> 13)) >> 3) +
                ((bme280Calibration.DigP2() * var1) >> 1)) >> 18;
        var1 = ((32768 + var1) * bme280Calibration.DigP1()) >> 15;
        if (var1 == 0) {
            return 0; // to avoid exception caused by division by zero
        }
        long pressure = ((1048576 - rawData) - (var2 >> 12)) * 3125;
        pressure = pressure < 0x80000000 ? (pressure << 1) / var1 : (pressure / var1) * 2;
        var1  = (bme280Calibration.DigP9()*(((pressure >> 3)*(pressure >> 3)) >> 13)) >> 12;
        var2 = ((pressure >> 2) * bme280Calibration.DigP8()) >> 13;
        pressure = pressure + ((var1 + var2 + bme280Calibration.DigP7()) >> 4);

        return pressure;
    }

    /**
     * Returns humidity in %RH as unsigned 32 bit integer.
     * Output value of “47445” represents 47445 = 47.445 %RH.
     * @return current humidity
     */
    public long getHumidity() {
        final long HUMIDITY_MAX = 102400;

        final long msb = bme280IO.readRegister(Bme280Registers.HUMIDITY_REG);
        final long lsb = bme280IO.readRegister(Bme280Registers.HUMIDITY_REG+1);
        final long rawData = (msb << 8) | lsb;

        // Some magic below from oficial BME280.c file from Bosch
        final long var1 = bme280Calibration.t_fine - 76800;
        long var2 = rawData << 14;
        long var3 = bme280Calibration.DigH4() << 20;
        long var4 = bme280Calibration.DigH5() * var1;
        long var5 = (((var2 - var3) - var4) + 16384) >> 15;

        var2 = (var1 * bme280Calibration.DigH6()) >> 10;
        var3 = (var1 * bme280Calibration.DigH3()) >> 11;
        var4 = ((var2*(var3 + 32768)) >> 10) + 2097152;

        var2 = (var4*bme280Calibration.DigH2()+8192) >> 14;
        var3 = var5 * var2;
        var4 = ((var3 >> 15) * (var3 >> 15)) >> 7;
        var5 = var3 - ((var4 * bme280Calibration.DigH1()) >> 4);
    
        var5 = (var5 < 0 ? 0 : var5);
        var5 = (var5 > 419430400 ? 419430400 : var5);
        final long humidity = var5 >> 12;

        return humidity > HUMIDITY_MAX ? HUMIDITY_MAX : humidity; 
    }
}
