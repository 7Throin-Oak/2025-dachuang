from ati_axia80_ethernet_python import ForceTorqueSensorDriver
import time

# Example usage
if __name__ == "__main__":
    sensor_ip = "192.168.1.102" # your ip address probably 192.168.1.1
    driver = ForceTorqueSensorDriver(sensor_ip)

    try:
        driver.start()

        print("Calibration data:")
        print(driver.get_calibration_data())#传感器校准
        print() 
        
        print("Sensor configuration:")
        print(driver.get_sensor_configuration())#传感器配置
        print()
        i=0
        s=time.time()
        while time.time()-s<=10:
            st=time.time()
            reading = driver.get_wrench()
            if reading:
                print(reading)
            i=i+1
            print(i)
            while time.time()- st< 0.001:
                pass


    except KeyboardInterrupt:
        print("Stopping the driver...")
    finally:
        driver.stop()
