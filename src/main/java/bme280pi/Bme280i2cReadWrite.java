package bme280pi;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import jdk.dio.i2cbus.I2CDevice;

/**
 * Basic read-write data from sensor registers
 */
public class Bme280i2cReadWrite {
    
    /**
     * One Byte.
     */
    private final static int ONE_BYTE = 1;

    /**
     * i2c Device driver
     */
    private I2CDevice i2CDevice;

    /**
     * Default constructor.
     * @param i2CDevice - i2c device driver
     */
    public Bme280i2cReadWrite(I2CDevice i2CDevice) {
        this.i2CDevice = i2CDevice;
    }

    /**
     * Read one byte of data from Register.
     * @param register - register of BME280 to be read (0xD0, 0xE0, etc)
     * @return bme280 register value (unsigned)
     */
    public long readRegister(int register) {
        return readRegister(register, ONE_BYTE);
    }

    /**
     * Read number of bytes from bme280 register.
     * @param register - register to be read
     * @param size     - number of bytes to be read
     * @return bme280 register data as unsigned value
     */
    public long readRegister(int register, int size) {
        try {
            // assert based on Table 18: memory map from BME280 datasheet
            assert (register >= Bme280Registers.DIG_T1_REG && register <= Bme280Registers.DIG_H1_REG) ||
                    (register >= Bme280Registers.DIG_H2_REG && register <= Bme280Registers.CTRL_HUM_REG) ||
                    register == Bme280Registers.CHIP_ID_REG||
                    register == Bme280Registers.CTRL_HUM_REG ||
                    register == Bme280Registers.STATUS_REG ||
                    register == Bme280Registers.CTRL_MES_REG ||
                    register == Bme280Registers.CONFIG_REG ||
                    register == Bme280Registers.PRESSURE_REG ||
                    register == Bme280Registers.TEMPERATURE_REG ||
                    register == Bme280Registers.HUMIDITY_REG;

            // See Table 16. Compencation parameter storage, naming and data type
            final boolean signed = (register == Bme280Registers.DIG_T2_REG+1) ||
                                   (register == Bme280Registers.DIG_T3_REG+1) ||
                                   (register == Bme280Registers.DIG_P2_REG+1) ||
                                   (register == Bme280Registers.DIG_P3_REG+1) ||
                                   (register == Bme280Registers.DIG_P4_REG+1) ||
                                   (register == Bme280Registers.DIG_P5_REG+1) ||
                                   (register == Bme280Registers.DIG_P6_REG+1) ||
                                   (register == Bme280Registers.DIG_P7_REG+1) ||
                                   (register == Bme280Registers.DIG_P8_REG+1) ||
                                   (register == Bme280Registers.DIG_P9_REG+1) ||
                                   (register == Bme280Registers.DIG_H2_REG+1) ||
                                   (register == Bme280Registers.DIG_H4_REG+1) ||
                                   (register == Bme280Registers.DIG_H5_REG+1) ||
                                   (register == Bme280Registers.DIG_H6_REG);

            ByteBuffer dst = ByteBuffer.allocateDirect(size);
            int nBytesRead = i2CDevice.read(register, size, 0, dst);

            assert nBytesRead == size;

            return byteBufferToLong(dst, signed);
        } catch (Exception e) {
            e.printStackTrace();
        }
        return -1;
    }

    /**
     * Write one byte of data into bme280 register.
     * @param register - register to be write (0xF5, 0xF4, 0xF2, 0xE0)
     * @param data     - data to be written (one byte)
     */
    public void writeRegister(int register, long data) {
        writeRegister(register, data, ONE_BYTE);
    }

    /**
     * Write up to 8 bytes into bme280 register.
     * @param register - register to be write (0xF5, 0xF4, 0xF2, 0xE0)
     * @param data     - up to 8 bytes to be written
     * @param size     - size (1 to 8)
     */
    public void writeRegister(int register, long data, int size) {
        try {
            assert Bme280Registers.CONFIG_REG == register || 
                    Bme280Registers.CTRL_MES_REG == register || 
                    Bme280Registers.CTRL_HUM_REG == register || 
                    Bme280Registers.RESET_REG == register;

            ByteBuffer src = longToByteBuffer(data);

            int nBytesWrite = i2CDevice.write(register, size, src);
            assert nBytesWrite == size;
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Convert Byte Buffer to long (8 byte???).
     * @param buffer - buffer to convert
     * @param signed - current register is signed
     * @return - converted value
     */
    private long byteBufferToLong(ByteBuffer buffer, boolean signed) {
        long result = 0L;

        // create array of Bytes with size of number of Bytes in ByteBuffer
        final byte arr[] = new byte[buffer.position()]; 
        buffer.rewind(); // move read pointer at the beggining of ByteBuffer
        buffer.get(arr); // copy all data from ByteBuffer to array of Bytes
        
        // Convert array of Bytes to long
        for (int i=0; i<arr.length; i++) {
            result += (arr[i]<0 && !signed?0xFF+arr[i]:arr[i]) << i*8;
        }

        return result;
    }

    /**
     * Convert long (8 byte ???) to ByteBuffer.
     * @param value - long value
     * @return - converted value
     */
    private ByteBuffer longToByteBuffer(final long value) {
        ByteBuffer result = ByteBuffer.allocateDirect(8); // ??? allocate to size of long
        for (int i = 0; i < 8; i++) {
            if (result.order() == ByteOrder.BIG_ENDIAN) {
                result.put((byte) ((value >> 8 * i) & 0x00FF));
            } else {
                result.put((byte) ((value & (0xFF << 8 * (8 - i - 1))) >> 8 * (8 - i - 1))); // 8-i-1 size of long???
            }
        }
        result.rewind();
        return result;
    }
}
