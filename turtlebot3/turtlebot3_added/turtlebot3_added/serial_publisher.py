#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import json

class ArduinoSensorReader(Node):
    def __init__(self):
        super().__init__('arduino_sensor_reader')
        
        # Crear el publicador
        self.publisher = self.create_publisher(String, 'external_sensor_data', 10)
        
        # Configurar la conexión serial
        # Ajusta el puerto según tu configuración
        try:
            self.serial_conn = serial.Serial(
                port='/dev/ttyACM1', 
                baudrate=230400,
                timeout=1.0
            )
            self.get_logger().info('Conexión serial establecida exitosamente')
        except serial.SerialException as e:
            self.get_logger().error(f'Error al abrir el puerto serial: {str(e)}')
            return

        # Crear timer para leer datos
        self.timer = self.create_timer(0.1, self.read_and_publish)  # 10Hz

    def read_and_publish(self):
        try:
            if self.serial_conn.in_waiting:
                # Leer línea del Arduino
                line = self.serial_conn.readline().decode('utf-8').strip()
                
                # Ignorar líneas de debug
                if line.startswith('[DEBUG]'):
                    return
                
                # Procesar datos (formato esperado: "peso1,peso2")
                try:
                    peso1, peso2 = map(float, line.split(','))
                    
                    # Crear mensaje
                    msg = String()
                    msg.data = json.dumps([peso1, peso2])
                    
                    # Publicar mensaje
                    self.publisher.publish(msg)
                    self.get_logger().info(f'Datos publicados: {msg.data}')
                    
                except ValueError as e:
                    self.get_logger().warn(f'Error al procesar datos: {str(e)}')
                
        except serial.SerialException as e:
            self.get_logger().error(f'Error de comunicación serial: {str(e)}')

    def __del__(self):
        if hasattr(self, 'serial_conn'):
            self.serial_conn.close()

def main(args=None):
    rclpy.init(args=args)
    
    arduino_reader = ArduinoSensorReader()
    
    try:
        rclpy.spin(arduino_reader)
    except KeyboardInterrupt:
        pass
    finally:
        arduino_reader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
