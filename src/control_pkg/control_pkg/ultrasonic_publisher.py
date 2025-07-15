import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial
import time

class UltrasonicPublisher(Node):
    def __init__(self):
        # 'arduino_ultrasonic_publisher' 노드 초기화
        super().__init__('arduino_ultrasonic_publisher')

        # 런치 파일에서 파라미터를 가져오도록 선언
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('num_sensors', 10)

        # 선언된 파라미터 값 가져오기
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.num_sensors = self.get_parameter('num_sensors').get_parameter_value().integer_value

        # 각 센서에 대한 퍼블리셔 생성
        self.publishers_ = []
        for i in range(self.num_sensors):
            topic_name = f'/ultrasonic/sensor_{i}'
            self.publishers_.append(self.create_publisher(Range, topic_name, 10))
            self.get_logger().info(f'Publishing to {topic_name}')

        # 시리얼 포트 연결 시도
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
            time.sleep(2) # 연결 안정화를 위한 대기 시간
            self.get_logger().info(f'Successfully connected to serial port: {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port {self.serial_port}: {e}')
            rclpy.shutdown()
            return

        # 0.1초마다 read_and_publish_data 함수 실행
        self.timer = self.create_timer(0.1, self.read_and_publish_data)

    def read_and_publish_data(self):
        if self.serial_conn.in_waiting > 0:
            try:
                # 아두이노로부터 한 줄의 데이터 읽기 (예: "0,15.2;1,34.5;...")
                line = self.serial_conn.readline().decode('utf-8').strip()
                self.get_logger().debug(f'Raw data from Arduino: "{line}"')

                # 데이터 파싱 (예: "센서인덱스,거리값")
                sensor_data = line.split(';')
                for data in sensor_data:
                    if ',' in data:
                        parts = data.split(',')
                        sensor_index = int(parts[0])
                        distance_cm = float(parts[1])

                        if 0 <= sensor_index < self.num_sensors:
                            msg = Range()
                            msg.header.stamp = self.get_clock().now().to_msg()
                            # 각 센서의 위치에 맞게 frame_id 설정 가능
                            msg.header.frame_id = f'ultrasonic_sensor_{sensor_index}_frame'
                            msg.radiation_type = Range.ULTRASOUND
                            msg.field_of_view = 0.26  # 약 15도 (라디안)
                            msg.min_range = 0.02  # 2cm
                            msg.max_range = 4.00  # 4m
                            msg.range = distance_cm / 100.0  # cm를 m로 변환

                            # 해당 센서의 퍼블리셔로 메시지 발행
                            self.publishers_[sensor_index].publish(msg)
                        else:
                            self.get_logger().warn(f'Sensor index {sensor_index} out of range.')

            except (ValueError, IndexError) as e:
                self.get_logger().warn(f'Could not parse serial data: "{line}". Error: {e}')
            except Exception as e:
                self.get_logger().error(f'An unexpected error occurred: {e}')

    def destroy_node(self):
        # 노드 종료 시 시리얼 연결 닫기
        if hasattr(self, 'serial_conn') and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info('Serial port closed.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()