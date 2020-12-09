package bme280pi;

/**
 * BME 280 Registers address.
 * (see Cahpter 5 in Final Datasheet Bosch BME280 Environmental sensor).
 * 
 * To dump all registers type: sudo i2cdump -y 1 0x76
 */
public final class Bme280Registers {
    /**
     * Humidity register address.
     * 0xFE - humidity lsb (read-only)
     * 0xFD - humidity msb (read-only)
     */
    public static final int HUMIDITY_REG = 0xFD;

    /**
     * Temperature register address.
     * 0xFC - temperature xlsb (read-only)
     * 0xFB - temperature lsb (read-only)
     * 0xFA - temperature msb (read-only)
     */
    public static final int TEMPERATURE_REG = 0xFA;

    /**
     * Humidity register address.
     * 0xF9 - pressure xlsb (used only bits [7:4]) (read-only)
     * 0xF8 - pressure lsb (read-only)
     * 0xF7 - pressure msb (read-only)
     */
    public static final int PRESSURE_REG = 0xF7;

    /**
     * Config register address.
     * 0xF5: [7:5] - [2:0] t_sb todo update
     *       [4:2] - [2:0] Filter
     *       [1:1] - not used (reserved. not changed)
     *       [0:0] - spi_w_en (should be 0 since i2c used)
     */
    public static final int CONFIG_REG = 0xF5;

    /**
     * Control temperature and pressure mesurments register address.
     * 0xF4: [7:5] - [2:0] osrs_t
     *       [4:2] - [2:0] osrs_p
     *       [1:0] - [1:0] mode
     */
    public static final int CTRL_MES_REG = 0xF4;

    /**
     * Status register address.
     * 0xF3: [7:4] and [2:1] - not used (reserved. not changed)
     *       [3:3] - measuring (read-only)
     *       [0:0] - im_update (read-only)
     */
    public static final int STATUS_REG = 0xF3;

    /**
     * Control humidity mesurments register address.
     * 0xF2: [7:3] - not used (reserved. not changed)
     *       [2:0] - osrs_h
     */
    public static final int CTRL_HUM_REG = 0xF2;

    /**
     * Sensor reset register address.
     * 0xE0 - reset (write only).
     */
    public static final int RESET_REG = 0xE0;

    /**
     * Sensor chip id register address
     * 0xD0 - chip id (read-only)
     */
    public static final int CHIP_ID_REG = 0xD0;

    // Calibration registers addresses (read-only)
    /**
     * BME 280 temperature Calibration DigT1 register address.
     * 0x88/0x89 unsigned short (2 bytes).
     */
    public static final int DIG_T1_REG = 0x88;
    /**
     * BME 280 temperature Calibration DigT2 register address.
     * 0x8A/0x8B signed short (2 bytes).
     */
    public static final int DIG_T2_REG = 0x8A;

    /**
     * BME 280 temperature Calibration DigT3 register address.
     * 0x8C/0x8D signed short (2 bytes).
     */
    public static final int DIG_T3_REG = 0x8C;

    /**
     * BME 280 pressure Calibration DigP1 register address.
     * 0x8E/0x8F signed short (2 bytes).
     */
    public static final int DIG_P1_REG = 0x8E;

    /**
     * BME 280 pressure Calibration DigP2 register address.
     * 0x90/0x91 signed short (2 bytes).
     */
    public static final int DIG_P2_REG = 0x90;

    /**
     * BME 280 pressure Calibration DigP3 register address.
     * 0x92/0x93 signed short (2 bytes).
     */
    public static final int DIG_P3_REG = 0x92;

    /**
     * BME 280 pressure Calibration DigP4 register address.
     * 0x94/0x95 signed short (2 bytes).
     */
    public static final int DIG_P4_REG = 0x94;

    /**
     * BME 280 pressure Calibration DigP5 register address.
     * 0x96/0x97 signed short (2 bytes).
     */
    public static final int DIG_P5_REG = 0x96;

    /**
     * BME 280 pressure Calibration DigP6 register address.
     * 0x98/0x99 signed short (2 bytes).
     */
    public static final int DIG_P6_REG = 0x98;

    /**
     * BME 280 pressure Calibration DigP7 register address.
     * 0x9A/0x9B signed short (2 bytes).
     */
    public static final int DIG_P7_REG = 0x9A;

    /**
     * BME 280 pressure Calibration DigP8 register address.
     * 0x9C/0x9D signed short (2 bytes).
     */
    public static final int DIG_P8_REG = 0x9C;

    /**
     * BME 280 pressure Calibration DigP9 register address.
     * 0x9E/0x9F signed short (2 bytes).
     */
    public static final int DIG_P9_REG = 0x9E;

    /**
     * BME 280 pressure Calibration DigH1 register address.
     * 0xA1 unsigned char (1 byte).
     */
    public static final int DIG_H1_REG = 0xA1;

    /**
     * BME 280 pressure Calibration DigH2 register address.
     * 0xE1/0xE2 signed short (2 bytes).
     */
    public static final int DIG_H2_REG = 0xE1;

    /**
     * BME 280 pressure Calibration DigH3 register address.
     * 0xE3 signed short (1 byte).
     */
    public static final int DIG_H3_REG = 0xE3;

    /**
     * BME 280 pressure Calibration DigH4 register address.
     * 0xE4[11:4]/0xE5[3:0] signed short (12 bits).
     */
    public static final int DIG_H4_REG = 0xE4;

    /**
     * BME 280 pressure Calibration DigH5 register address.
     * 0xE5[7:4]/0xE6 signed short (12 bits).
     */
    public static final int DIG_H5_REG = 0xE5;

    /**
     * BME 280 pressure Calibration DigH6 register address.
     * 0xE7 signed char (1 byte).
     */
    public static final int DIG_H6_REG = 0xE7;
}
