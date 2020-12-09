package bme280pi;

/**
 * BME 280 Compensation parameter storage.
 */
public final class Bme280CalibrationParams {
    /**
     * Temperature T1 Compensation parameter. 
     * Address: 0x88/0x89 dig_T1[7:0]/[15:8] unsigned short
     */
    private int digT1;

    /**
     * Temperature T2 Compensation parameter. 
     * Address: 0x8A/0x8B dig_T2[7:0]/[15:8] signed short
     */
    private int digT2;

    /**
     * Temperature T3 Compensation parameter. 
     * Address: 0x8C/0x8D dig_T3[7:0]/[15:8] signed short
     */
    private int digT3;

    /**
     * Temperature P1 Compensation parameter. 
     * Address: 0x8E/0x8F dig_P1[7:0]/[15:8] unsigned short
     */
    private int digP1;

    /**
     * Temperature P2 Compensation parameter. 
     * Address: 0x90/0x91 dig_P2[7:0]/[15:8] signed short
     */
    private int digP2;

    /**
     * Temperature P3 Compensation parameter. 
     * Address: 0x92/0x93 dig_P3[7:0]/[15:8] signed short
     */
    private int digP3;

    /**
     * Temperature P4 Compensation parameter. 
     * Address: 0x94/0x95 dig_P4[7:0]/[15:8] signed short
     */
    private int digP4;

    /**
     * Temperature P5 Compensation parameter. 
     * Address: 0x96/0x97 dig_P5[7:0]/[15:8] signed short
     */
    private int digP5;

    /**
     * Temperature P6 Compensation parameter. 
     * Address: 0x98/0x99 dig_P6[7:0]/[15:8] signed short
     */
    private int digP6;

    /**
     * Temperature P7 Compensation parameter. 
     * Address: 0x9A/0x9B dig_P7[7:0]/[15:8] signed short
     */
    private int digP7;

    /**
     * Temperature P8 Compensation parameter. 
     * Address: 0x9C/0x9D dig_P8[7:0]/[15:8] signed short
     */
    private int digP8;

    /**
     * Temperature P9 Compensation parameter. 
     * Address: 0x9E/0x9F dig_P9[7:0]/[15:8] signed short
     */
    private int digP9;

    /**
     * Temperature H1 Compensation parameter. 
     * Address: 0xA1 dig_H1[7:0] unsigned char
     */
    private int digH1;
    
    /**
     * Temperature H2 Compensation parameter. 
     * Address: 0xE1/0xE2 dig_H2[7:0]/[15:8] signed short
     */
    private int digH2;

    /**
     * Temperature H3 Compensation parameter. 
     * Address: 0xE3 dig_H3[7:0] unsigned char
     */
    private int digH3;

    /**
     * Temperature H4 Compensation parameter. 
     * Address: 0xE4/0xE5[3:0] dig_H4[11:4]/[3:0] signed short
     */
    private int digH4;

    /**
     * Temperature H5 Compensation parameter. 
     * Address: 0xE5[7:4]/0xE6 dig_H5[3:0]/[11:4] signed short
     */
    private int digH5;

    /**
     * Temperature H6 Compensation parameter. 
     * Address: 0xE7 dig_H6 signed char
     */
    private int digH6;

    /**
     * Compencation temprature used on Humidity and Pressure calculations.
     */
    public long t_fine;

    /**
     * Initialize Calibration parameters.
     * @param bme280Sensor - BME 280 Sensor to read Calibration data
     */
    public Bme280CalibrationParams(Bme280Sensor bme280Sensor) {
        digT1 = bme280Sensor.readCalibrationData(Bme280Registers.DIG_T1_REG) | 
                (bme280Sensor.readCalibrationData(Bme280Registers.DIG_T1_REG + 1) << 8);
        digT2 = bme280Sensor.readCalibrationData(Bme280Registers.DIG_T2_REG) | 
                (bme280Sensor.readCalibrationData(Bme280Registers.DIG_T2_REG + 1) << 8);
        digT3 = bme280Sensor.readCalibrationData(Bme280Registers.DIG_T3_REG) |
                (bme280Sensor.readCalibrationData(Bme280Registers.DIG_T3_REG + 1) << 8);

        digP1 = bme280Sensor.readCalibrationData(Bme280Registers.DIG_P1_REG) | 
                (bme280Sensor.readCalibrationData(Bme280Registers.DIG_P1_REG + 1) << 8);
        digP2 = bme280Sensor.readCalibrationData(Bme280Registers.DIG_P2_REG) |
                (bme280Sensor.readCalibrationData(Bme280Registers.DIG_P2_REG + 1) << 8);
        digP3 = bme280Sensor.readCalibrationData(Bme280Registers.DIG_P3_REG) |
                (bme280Sensor.readCalibrationData(Bme280Registers.DIG_P3_REG + 1) << 8);
        digP4 = bme280Sensor.readCalibrationData(Bme280Registers.DIG_P4_REG) |
                (bme280Sensor.readCalibrationData(Bme280Registers.DIG_P4_REG + 1) << 8);
        digP5 = bme280Sensor.readCalibrationData(Bme280Registers.DIG_P5_REG) |
                (bme280Sensor.readCalibrationData(Bme280Registers.DIG_P5_REG + 1) << 8);
        digP6 = bme280Sensor.readCalibrationData(Bme280Registers.DIG_P6_REG) |
                (bme280Sensor.readCalibrationData(Bme280Registers.DIG_P6_REG + 1) << 8);
        digP7 = bme280Sensor.readCalibrationData(Bme280Registers.DIG_P7_REG) |
                (bme280Sensor.readCalibrationData(Bme280Registers.DIG_P7_REG + 1) << 8);
        digP8 = bme280Sensor.readCalibrationData(Bme280Registers.DIG_P8_REG) |
                (bme280Sensor.readCalibrationData(Bme280Registers.DIG_P8_REG + 1) << 8);
        digP9 = bme280Sensor.readCalibrationData(Bme280Registers.DIG_P9_REG) |
                (bme280Sensor.readCalibrationData(Bme280Registers.DIG_P9_REG + 1) << 8);

        digH1 = bme280Sensor.readCalibrationData(Bme280Registers.DIG_H1_REG);
        digH2 = bme280Sensor.readCalibrationData(Bme280Registers.DIG_H2_REG) |
                (bme280Sensor.readCalibrationData(Bme280Registers.DIG_H2_REG + 1) << 8);

        digH3 = bme280Sensor.readCalibrationData(Bme280Registers.DIG_H3_REG);

        digH4 = (bme280Sensor.readCalibrationData(Bme280Registers.DIG_H4_REG) << 4) |
                (bme280Sensor.readCalibrationData(Bme280Registers.DIG_H5_REG) & 0x0F);

        digH5 = ((bme280Sensor.readCalibrationData(Bme280Registers.DIG_H5_REG) & 0xF0) >> 4) | 
                (bme280Sensor.readCalibrationData(Bme280Registers.DIG_H5_REG + 1) << 4);
        digH6 = bme280Sensor.readCalibrationData(Bme280Registers.DIG_H6_REG);

        t_fine = 0;
    }

    /**
     * get Calibration DigT1.
     * @return digT1
     */
    public int DigT1() {
        return digT1;
    }

    /**
     * get Calibration DigT2.
     * @return digT2
     */
    public int DigT2() {
        return digT2;
    }

    /**
     * get Calibration DigT3.
     * @return digT3
     */
    public int DigT3() {
        return digT3;
    }

    /**
     * get Calibration DigP1.
     * @return digP1
     */
    public int DigP1() {
        return digP1;
    }

    /**
     * get Calibration DigP2.
     * @return digP2
     */
    public int DigP2() {
        return digP2;
    }

    /**
     * get Calibration DigP3.
     * @return digP3
     */
    public int DigP3() {
        return digP3;
    }

    /**
     * get Calibration DigP4.
     * @return digP4
     */
    public int DigP4() {
        return digP4;
    }

    /**
     * get Calibration DigP5.
     * @return digP5
     */
    public int DigP5() {
        return digP5;
    }

    /**
     * get Calibration DigP6.
     * @return digP6
     */
    public int DigP6() {
        return digP6;
    }

    /**
     * get Calibration DigP7.
     * @return digP7
     */
    public int DigP7() {
        return digP7;
    }

    /**
     * get Calibration DigP8.
     * @return digP8
     */
    public int DigP8() {
        return digP8;
    }

    /**
     * get Calibration DigP9.
     * @return digP9
     */
    public int DigP9() {
        return digP9;
    }

    /**
     * get Calibration DigH1.
     * @return digH1
     */
    public int DigH1() {
        return digH1;
    }

    /**
     * get Calibration DigH2.
     * @return digH2
     */
    public int DigH2() {
        return digH2;
    }

    /**
     * get Calibration DigH3.
     * @return digH3
     */
    public int DigH3() {
        return digH3;
    }

    /**
     * get Calibration DigH4.
     * @return digH4
     */
    public int DigH4() {
        return digH4;
    }

    /**
     * get Calibration DigH5.
     * @return digH5
     */
    public int DigH5() {
        return digH5;
    }

    /**
     * get Calibration DigH6.
     * @return digH6
     */
    public int DigH6() {
        return digH6;
    }
}
