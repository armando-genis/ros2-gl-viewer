#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException
import yaml
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy



class TFFrameMonitor(Node):
    def __init__(self):
        super().__init__('tf_frame_monitor')
        
        # Use a callback group for the timer to prevent blocking
        self.callback_group = ReentrantCallbackGroup()

        self.tf_qos = QoSProfile(
            depth=100,
            durability=QoSDurabilityPolicy.VOLATILE,  # Try VOLATILE instead of TRANSIENT_LOCAL
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        
        # Create TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, qos=self.tf_qos)

        
        # Set up timer for periodic frame checks - 1 Hz
        self.timer = self.create_timer(1.0, self.check_frames, callback_group=self.callback_group)
        
        # Track frames we've seen
        self.known_frames = set()
        self.frame_connectivity = {}
        
        # Counter for spin cycles
        self.spin_count = 0
        
        self.get_logger().info('TF Frame Monitor initialized. Checking frames every second...')
    
    def check_frames(self):
        """Check available frames and their connectivity"""
        self.spin_count += 1
        self.get_logger().info(f'Spin cycle {self.spin_count} completed')
        
        # Get all frames as YAML
        try:
            frames_yaml = self.tf_buffer.all_frames_as_yaml()
            frames_dict = yaml.safe_load(frames_yaml)
            
            if frames_dict:
                # Extract frame IDs
                frame_ids = set(frames_dict.keys())
                
                # Check for new frames
                new_frames = frame_ids - self.known_frames
                if new_frames:
                    self.get_logger().info(f'New frames discovered: {", ".join(new_frames)}')
                
                # Update known frames
                self.known_frames = frame_ids
                
                # Display complete tree
                self.get_logger().info(f'=== TF Tree ({len(frame_ids)} frames) ===')
                self.get_logger().info(frames_yaml)
                
                # Test connectivity between frames
                self.check_frame_connectivity(list(frame_ids))
            else:
                self.get_logger().warn('No frames found in the TF tree')
        
        except Exception as e:
            self.get_logger().error(f'Error getting frames: {str(e)}')
    
    def check_frame_connectivity(self, frames):
        """Test connectivity between all pairs of frames"""
        # Only test new pairs to avoid spam
        for source in frames:
            for target in frames:
                if source != target:
                    pair = (source, target)
                    
                    if pair not in self.frame_connectivity:
                        try:
                            # Try with and without leading slashes
                            variants = [
                                (source, target),
                                ('/' + source, target),
                                (source, '/' + target),
                                ('/' + source, '/' + target)
                            ]
                            
                            can_transform = False
                            for src, tgt in variants:
                                try:
                                    if self.tf_buffer.can_transform(tgt, src, rclpy.time.Time()):
                                        can_transform = True
                                        # Also try to actually get the transform
                                        transform = self.tf_buffer.lookup_transform(tgt, src, rclpy.time.Time())
                                        self.get_logger().info(f'Transform {src} -> {tgt}: OK')
                                        break
                                except TransformException:
                                    pass
                            
                            if not can_transform:
                                self.get_logger().warn(f'Transform {source} -> {target}: FAILED (tried with/without slashes)')
                            
                            self.frame_connectivity[pair] = can_transform
                            
                        except Exception as e:
                            self.get_logger().error(f'Error checking transform {source} -> {target}: {str(e)}')
    
    def run_tf2_echo_test(self, source="base_link", target="zedd"):
        """Simulate the tf2_echo tool's behavior"""
        self.get_logger().info(f'=== Running tf2_echo test: {source} -> {target} ===')
        
        # Try with and without leading slashes
        variants = [
            (source, target),
            ('/' + source, target),
            (source, '/' + target),
            ('/' + source, '/' + target)
        ]
        
        for src, tgt in variants:
            try:
                self.get_logger().info(f'Testing: {src} -> {tgt}')
                can_transform = self.tf_buffer.can_transform(tgt, src, rclpy.time.Time())
                
                if can_transform:
                    transform = self.tf_buffer.lookup_transform(tgt, src, rclpy.time.Time())
                    trans = transform.transform.translation
                    rot = transform.transform.rotation
                    
                    self.get_logger().info(f'Transform FOUND with variant: {src} -> {tgt}')
                    self.get_logger().info(f'Translation: [{trans.x}, {trans.y}, {trans.z}]')
                    self.get_logger().info(f'Rotation: [{rot.x}, {rot.y}, {rot.z}, {rot.w}]')
                    return True
                else:
                    self.get_logger().warn(f'Can\'t transform with variant: {src} -> {tgt}')
            
            except Exception as e:
                self.get_logger().warn(f'Exception with variant {src} -> {tgt}: {str(e)}')
        
        self.get_logger().error(f'Failed to find transform between {source} and {target} with any variant')
        return False


def main():
    rclpy.init()
    
    # Create and initialize the node
    monitor = TFFrameMonitor()
    
    # Sleep to allow initial discovery
    time.sleep(2.0)
    
    # Use MultiThreadedExecutor for better performance
    executor = MultiThreadedExecutor()
    executor.add_node(monitor)
    
    try:
        # Run the executor in a separate thread
        import threading
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        # Main loop for user interaction
        print("TF Frame Monitor running. Press Ctrl+C to exit.")
        print("Commands: 'echo [source] [target]' to test specific transforms")
        print("          'q' to quit")
        
        while rclpy.ok():
            try:
                cmd = input("> ")
                if cmd.lower() == 'q':
                    break
                
                if cmd.startswith('echo'):
                    parts = cmd.split()
                    if len(parts) == 3:
                        source = parts[1]
                        target = parts[2]
                        monitor.run_tf2_echo_test(source, target)
                    else:
                        print("Usage: echo [source_frame] [target_frame]")
            except KeyboardInterrupt:
                break
    
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        rclpy.shutdown()
        print("Monitor stopped.")


if __name__ == '__main__':
    main()