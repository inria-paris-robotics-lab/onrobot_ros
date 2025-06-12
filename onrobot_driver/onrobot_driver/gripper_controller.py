import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import String
from ur_msgs.srv import SetIO
from ur_msgs.msg import ToolDataMsg, IOStates
import time

class OnRobotDriver(Node):
    def __init__(self):
        super().__init__('onrobot_driver_test')
        self._tool_voltage = None
        self._position_voltage = None
        self._ready = False
        self._state = None  # Current state of the gripper (0 for open, 1 for closed)
        self._max_position_voltage = 10.0  # Max voltage for the position sensor pour le rg2 !!!!!!
        self.get_logger().info('OnRobot Driver Test Node has been started.')
        self._set_io = self.create_client(SetIO, '/left_io_and_status_controller/set_io')
        self._states_io_sub = self.create_subscription(IOStates, "/left_io_and_status_controller/io_states", self._io_states_cb, 10)
        self.tool_data_sub = self.create_subscription(ToolDataMsg, "/left_io_and_status_controller/tool_data", self._tool_data_cb, 10)
        self._set_io.wait_for_service(timeout_sec=5.0)
        self._script_command_pub = self.create_publisher(String, "/left_urscript_interface/script_command", 10)
        self.enable()
        time.sleep(1.0)  # Wait for the gripper to be ready
        # self.close()  # Start with the gripper closed

    

    def timer_cb(self):
        """ ferme et active le gripper toutes les 10 secondes """
        if self._state:
            self.get_logger().info('Gripper is currently closed, opening it.')
            self.open()
        else:
            self.get_logger().info('Gripper is currently open, closing it.')
            self.close()
        
    

    def enable(self): #OK
        """Enable gripper.

        Set the UR tool voltage to 24V and wait until the gripper becomes ready.

        Raises:
            ROSException: timeout exception
        """
        self._set_digital_out(1,16, False) # Always start with open gripper.
        self._set_tool_voltage(24) # Turn on tool
        if self._wait_for(lambda: self._tool_voltage == 24 and self._ready):
            self.get_logger().info('Gripper is already enabled.')
            return

        # Switching the pin 16 from High to Low (after power on) is necessary to 'wake up' the RG6-V2 gripper.
        self._set_digital_out(1,16, True)
        self._set_digital_out(1,16, False)
        # The timeout is increased so the grippeur has time to open (if needed)
        if not self._wait_for(lambda: self._tool_voltage == 24 and self._ready, timeout=12.0):
            self.get_logger().error('Failed to enable gripper: tool voltage is not 24V or gripper is not ready.')
            return
        self.get_logger().info('Gripper enabled successfully.')

    def disable(self): #OK
        """Disable gripper.

        Set the tool voltage to zero.

        Raises:
            ROSException: timeout exception
        """
        self._set_tool_voltage(0)
        if not self._wait_for(lambda: self._tool_voltage == 0):
            raise self.get_logger().error('Failed to disable gripper: tool voltage is not 0V.')
        
    @property
    def is_ready(self): #OK
        """Gripper is ready to receive a command.

        Returns:
            bool -- ready flag
        """
        return self._ready

    @property
    def opening(self):
        """Fingers opening in percents.

        Returns:
            float -- opening 0..1 such that 0 - fully closed, 1 - fully opened
        """
        return max(0.0, min(1.0, self._position_voltage / self._max_position_voltage))


    def _set_tool_voltage(self, voltage): # OK
        if voltage not in [0, 24]:
            self.get_logger().error("Invalid voltage: must be 0 or 24V")
            return
        self._set_digital_out(4, 0, voltage)  # Set the tool voltage pin to high if voltage > 0
        if not self._wait_for(lambda: self._tool_voltage == voltage):
            raise self.get_logger().error('Failed to move gripper: gripper is not ready or state is not as expected.')
        # cmd = String()
        # cmd.data = '\n'.join([
        #     'sec MyProgram():',
        #     'set_tool_voltage({:d})'.format(voltage),
        #     'end'])
        # self._script_command_pub.publish(cmd)



    def open(self, low_force_mode=False, wait=True): #OK
        """Open the gripper.

        Keyword Arguments:
            low_force_mode {bool} -- use the low force mode (5N instead of 40N) (default: {False})
            wait {bool} -- wait until the end of the movement (default: {False})

        Raises:
            ROSException: timeout exception
        """
        self.get_logger().info('Opening gripper...')
        self._move(0, low_force_mode, wait)

    def close(self, low_force_mode=False, wait=True): #OK
        """Close the gripper.

        Keyword Arguments:
            low_force_mode {bool} -- use the low force mode (5N instead of 40N) (default: {False})
            wait {bool} -- wait until the end of the movement (default: {False})

        Raises:
            ROSException: timeout exception
        """
        self.get_logger().info('Closing gripper...')
        self._move(1, low_force_mode, wait)

    def _move(self, target, low_force_mode=False, wait=True): # OK
        self._set_digital_out(1,17, 1 if low_force_mode else 0)
        self._set_digital_out(1,16, target)
        self._ready = False
        if wait and not self._wait_for(lambda: self._ready and self._state == target):
            raise self.get_logger().error('Failed to move gripper: gripper is not ready or state is not as expected.')
        self.get_logger().info(f'Gripper moved to position {target} with low force mode {low_force_mode}.')
    
    def _set_digital_out(self, fun, pin, state): # OK
        req = SetIO.Request()
        req.fun = fun
        req.pin = pin
        req.state = float(state)
        self.future = self._set_io.call_async(req)
        print("sending request to set digital out")
        # rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def _tool_data_cb(self, tool_data):#OK
        self._tool_voltage = tool_data.tool_output_voltage
        self._position_voltage = tool_data.analog_input2
    
    def _io_states_cb(self, io_states): # OK
        for digital in io_states.digital_in_states:
            if digital.pin == 16:
                self._state = int(digital.state)
            elif digital.pin == 17:
                self._ready = bool(digital.state)

    def _wait_for(self, condition_fn, timeout=5.0): #OK
        start_time = self.get_clock().now().nanoseconds / 1e9
        deadline = start_time + timeout
        while True:
            if condition_fn():
                return True
            if (self.get_clock().now().nanoseconds / 1e9) > deadline:
                return False
            if not rclpy.ok():
                raise KeyboardInterrupt("rclpy shutdown")
            rclpy.spin_once(self, timeout_sec=0.01)



def main(args=None):
    rclpy.init(args=args)
    node = OnRobotDriverTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()