#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import serial
import struct
import time
import threading
import psutil
import os

class ImuUartPublisher(Node):
    def __init__(self):
        super().__init__('imu_uart_publisher')
        
        # Publisher for joint states
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # UART configuration
        self.uart_port = '/dev/ttyS2'
        self.baudrate = 115200
        self.packet_header = 0xAA55
        self.packet_footer = 0x55AA
        self.packet_size = 10
        
        # Thread-safe data storage
        self.latest_roll = 0.0
        self.latest_pitch = 0.0
        self.latest_yaw = 0.0
        self.data_lock = threading.Lock()
        self.packet_count = 0
        self.published_count = 0
        
        # Performance monitoring
        self.start_time = time.time()
        self.process = psutil.Process(os.getpid())
        
        # Initialize CPU measurements (establish baseline)
        psutil.cpu_percent(interval=None)
        self.process.cpu_percent(interval=None)
        time.sleep(0.1)
        
        # Rate calculation tracking
        self.last_packet_count = 0
        self.last_published_count = 0
        self.last_rate_calc_time = time.time()
        
        # Stats update interval
        self.stats_interval = 0.1
        
        # Initialize serial connection
        try:
            self.ser = serial.Serial(
                port=self.uart_port,
                baudrate=self.baudrate,
                timeout=0.1
            )
            self.ser.reset_input_buffer()
            time.sleep(0.5)
            self.get_logger().info(f'UART opened on {self.uart_port}')
            
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open UART: {e}')
            return
        
        # Start continuous reading thread
        self.running = True
        self.read_thread = threading.Thread(target=self.uart_read_loop, daemon=True)
        self.read_thread.start()
        
        # Create timer for publishing at 50Hz
        self.timer = self.create_timer(0.02, self.publish_callback)
        
        # Create timer for stats display
        self.stats_timer = self.create_timer(self.stats_interval, self.print_statistics)
    
    def get_system_stats(self):
        try:
            process_cpu = min(self.process.cpu_percent(interval=None), 100.0)
            process_mem_mb = self.process.memory_info().rss / 1024 / 1024
            system_cpu = psutil.cpu_percent(interval=None)
            system_mem = psutil.virtual_memory()
            system_swap = psutil.swap_memory()
            runtime = time.time() - self.start_time
            return {
                'process_cpu': process_cpu,
                'process_mem_mb': process_mem_mb,
                'system_cpu': system_cpu,
                'system_mem_percent': system_mem.percent,
                'system_mem_mb': system_mem.used / 1024 / 1024,
                'system_mem_total_mb': system_mem.total / 1024 / 1024,
                'system_swap_percent': system_swap.percent,
                'system_swap_mb': system_swap.used / 1024 / 1024,
                'runtime_seconds': runtime
            }
        except Exception as e:
            self.get_logger().error(f'Error in get_system_stats: {e}')
            return None
    
    def print_statistics(self):
        current_time = time.time()
        elapsed = current_time - self.last_rate_calc_time
        
        stats = self.get_system_stats()
        if stats is None:
            return
        
        runtime = stats['runtime_seconds']
        hours = int(runtime // 3600)
        minutes = int((runtime % 3600) // 60)
        seconds = int(runtime % 60)
        
        # Get current counts (thread-safe)
        with self.data_lock:
            current_packets = self.packet_count
            current_published = self.published_count
            roll = self.latest_roll
            pitch = self.latest_pitch
            yaw = self.latest_yaw
        
        # Calculate instantaneous rates
        uart_rate = (current_packets - self.last_packet_count) / elapsed if elapsed > 0 else 0
        ros_rate = (current_published - self.last_published_count) / elapsed if elapsed > 0 else 0
        
        # Update tracking variables
        self.last_packet_count = current_packets
        self.last_published_count = current_published
        self.last_rate_calc_time = current_time
        
        # Build compact output (3 lines)
        output = f"\r\033[K"
        
        # Line 1: Time and Process metrics
        output += f"[{hours:02d}:{minutes:02d}:{seconds:02d}] "
        output += f"Process: CPU {stats['process_cpu']:5.1f}% | RAM {stats['process_mem_mb']:5.1f}MB"
        
        # Line 2: System metrics
        output += f"\n\033[K"
        output += f"System:  CPU {stats['system_cpu']:5.1f}% | "
        output += f"RAM {stats['system_mem_percent']:4.1f}% ({stats['system_mem_mb']:.0f}/{stats['system_mem_total_mb']:.0f}MB)"
        
        # Add swap if being used
        if stats['system_swap_percent'] > 0.1:
            output += f" | Swap {stats['system_swap_percent']:4.1f}% ({stats['system_swap_mb']:.0f}MB)"
        
        # Line 3: Communication rates and attitude
        output += f"\n\033[K"
        output += f"Rates:   UART {uart_rate:6.1f}Hz | ROS {ros_rate:5.1f}Hz | "
        output += f"Attitude: R:{roll:6.2f}° P:{pitch:6.2f}° Y:{yaw:6.2f}°"
        
        # Move cursor up 2 lines
        output += "\033[2A"
        
        print(output, end='', flush=True)
    
    def find_header(self):
        """Find packet header for synchronization"""
        buffer = bytearray()
        timeout = 1000
        
        while timeout > 0 and self.running:
            byte = self.ser.read(1)
            if len(byte) == 0:
                timeout -= 1
                continue
                
            buffer.append(byte[0])
            if len(buffer) > 2:
                buffer.pop(0)
            
            if len(buffer) == 2:
                value = struct.unpack('<H', bytes(buffer))[0]
                if value == self.packet_header:
                    return True
        return False
    
    def uart_read_loop(self):
        """Continuous UART reading thread"""
        self.get_logger().info('UART reading thread started')
        
        if not self.find_header():
            self.get_logger().error('Failed to synchronize!')
            return
        
        self.get_logger().info('Synchronized with IMU stream')
        print("\n\n\n")
        
        error_count = 0
        
        while self.running:
            try:
                data = self.ser.read(self.packet_size - 2)
                
                if len(data) != self.packet_size - 2:
                    if self.find_header():
                        continue
                    break
                
                values = struct.unpack('<3hH', data)
                roll_raw, pitch_raw, yaw_raw, footer = values
                
                if footer != self.packet_footer:
                    error_count += 1
                    if self.find_header():
                        continue
                    break
                
                roll = roll_raw / 100.0
                pitch = pitch_raw / 100.0
                yaw = yaw_raw / 100.0
                
                with self.data_lock:
                    self.latest_roll = roll
                    self.latest_pitch = pitch
                    self.latest_yaw = yaw
                    self.packet_count += 1
                
                if not self.find_header():
                    break
                    
            except struct.error:
                error_count += 1
                if self.find_header():
                    continue
                break
            except Exception as e:
                print(f"\n\n\n")
                self.get_logger().error(f'Read error: {e}')
                break
        
        print(f"\n\n\n")
        self.get_logger().warn('UART reading thread stopped')
    
    def publish_callback(self):
        """Publish latest IMU data as joint states"""
        with self.data_lock:
            roll = self.latest_roll
            pitch = self.latest_pitch
            yaw = self.latest_yaw
        
        roll_rad = roll * 3.14159 / 180.0
        pitch_rad = pitch * 3.14159 / 180.0
        yaw_rad = yaw * 3.14159 / 180.0
        
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.name = ['roll_joint', 'pitch_joint', 'yaw_joint']
        msg.position = [roll_rad, pitch_rad, yaw_rad]
        
        self.publisher.publish(msg)
        
        with self.data_lock:
            self.published_count += 1
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        print(f"\n\n\n")
        self.running = False
        if hasattr(self, 'read_thread'):
            self.read_thread.join(timeout=2.0)
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImuUartPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
