package bme280pi;

/**
 * Hello world!
 *
 */
public class App {
    private final static int BME280_ADDR = 0x76;

    public static void main(String[] args) {
        
        long chipMode = 0;
        long oversampling = 0;
        System.out.println("Hello World!");
        Bme280Sensor bme280Sensor = new Bme280Sensor(BME280_ADDR);
        System.console().writer().println("Chip ID = " + bme280Sensor.getChipId());
        chipMode = bme280Sensor.getChipMode();
        System.console().writer().println("Chip Mode = " + chipMode);
        if (chipMode == 0) {
            bme280Sensor.setChipMode(0b11);
        }
        while (chipMode == 0) {
            chipMode = bme280Sensor.getChipMode();
        }
        oversampling = bme280Sensor.getOversampling(0);
        System.console().writer().println("Temperature oversampling = "+oversampling);
        if (oversampling == 0) {
            bme280Sensor.setOversampling(0, 1);
        }
        while (oversampling == 0) {
            oversampling = bme280Sensor.getOversampling(0);
        }

        oversampling = bme280Sensor.getOversampling(1);
        System.console().writer().println("Pressure oversampling = "+oversampling);
        if (oversampling == 0) {
            bme280Sensor.setOversampling(1, 1);
        }
        while (oversampling == 0) {
            oversampling = bme280Sensor.getOversampling(1);
        }
        oversampling = bme280Sensor.getOversampling(2);
        System.console().writer().println("Humidity oversampling = "+oversampling);
        if (oversampling == 0) {
            bme280Sensor.setOversampling(2, 1);
        }
        while (oversampling == 0) {
            oversampling = bme280Sensor.getOversampling(2);
        }
        
        System.console().writer().println("Chip Mode = " + chipMode);

        System.console().writer().println("Status = " + bme280Sensor.getStatus());
        long temp = 0;
        long press = 0;
        long humd = 0;
        long status = 0;
        for (int i = 0; i < 100; i++) {
            temp = bme280Sensor.getTemperature();
            press = bme280Sensor.getPressure();
            humd = bme280Sensor.getHumidity();
            status = bme280Sensor.getStatus();
            System.console().writer().println("Iteration = " + i);
            System.console().writer().println("Temperature = " + temp);
            System.console().writer().println("Pressure = " + press);
            System.console().writer().println("Humidity = " + humd);
            System.console().writer().println("Status = " + status);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }        
    }
}
