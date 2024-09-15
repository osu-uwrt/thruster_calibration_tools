#! /usr/bin/env python3

import time
from datetime import datetime
from statistics import mean
import csv
import yaml
import os

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from std_msgs.msg import Empty, Float32, Bool
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data

from thruster_cal_msgs.action import CalibrateThruster
from riptide_msgs2.msg import DshotCommand, DshotRPMFeedback, BatteryStatus

NEUTRAL_DSHOT = 0           # Dshot value which is "off" for the thrusters
DSHOT_PUB_PERIOD = 0.01    # Time in seconds between dshot timer publishes
DELAY_TIME = 5           # Delay in seconds between sending command and collecting data
N_SAMPLES = 15              # Number of samples to collect at each dshot value
COLLECTION_TIMEOUT = 5     # Allowed data collection time in seconds before timeout

NEGATIVE = False


class CalibrateThrusterAction(Node):

    def __init__(self):
        # Create Node
        super().__init__('calibrate_thruster')

        # Variables
        self.running = False
        self.collecting_data = False
        self.triggered = False
        self.dshot = NEUTRAL_DSHOT
        self.thruster_length = self.get_thruster_length()
        self.thruster_num = 0
        self.idle_current = 0
        self.present_current = 0
        self.battery_serial = None
        self.rpm = []
        self.force = []
        self.power = []

        # Create publishers and subscribers
        self.dshot_pub = self.create_publisher(
            DshotCommand, "command/thruster_rpm", qos_profile_sensor_data)
        self.calibrating_pub = self.create_publisher(
            Bool, "state/thruster_cal/calibrating", qos_profile_system_default)
        self.trigger_sub = self.create_subscription(
            Empty, "command/trigger", self.trigger_callback, qos_profile_system_default)
        self.thruster_rpm_sub = self.create_subscription(
            DshotRPMFeedback, "state/thrusters/rpm_complete", self.rpm_callback, qos_profile_sensor_data)
        self.thruster_force_sub = self.create_subscription(
            Float32, "force_gauge/force", self.force_callback, qos_profile_sensor_data)
        self.electrical_sub = self.create_subscription(
            BatteryStatus, "state/battery", self.electrical_callback, qos_profile_sensor_data)

        # Create action server
        self._action_server = ActionServer(
            self,
            CalibrateThruster,
            'calibrate_thruster',
            self.collect_data_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

        # Create timer to keep publishing the dshot command to prevent firmware from timing out
        self.timer = self.create_timer(
            DSHOT_PUB_PERIOD, self.publish_dshot_command)
        
        # Create timer to publish calibrating signal for gui
        self.pub_calibrating_timer = self.create_timer(
            1, self.publish_running)
    
    
    def publish_running(self):
        msg = Bool()
        msg.data = self.running
        self.calibrating_pub.publish(msg)
    
    
    def feedback_message(self, goal_handle, dshot, message):
        self.get_logger().info(message)
        feedback_msg = CalibrateThruster.Feedback()
        feedback_msg.current_dshot = dshot
        feedback_msg.message = message
        goal_handle.publish_feedback(feedback_msg)
    

    def collect_data_callback(self, goal_handle):
        # Method walks data through collection process and saves the result to csv file
        self.thruster_num = goal_handle.request.thruster_num
        
        # Wait for user to tare scale by reseting the arduino        
        self.feedback_message(goal_handle, 0, 
                              "Please ensure rpm echo node is running!\n" + 
                              "The background current will also be measured at this time\n" +
                              "Please reset arduino to tare the scale, publish command/trigger once complete")
        self.wait_for_input()

        # Start calibration, looping through dshot values and saving data
        self.feedback_message(goal_handle, 0, 'Thruster calibration starting...')
        self.idle_current = self.present_current
        flag = True
        rows_data = []
        flip_force = 1

        # generate the dshot values to measure at
        measurementValues = []
        measurementValuesNegative = []
        val = goal_handle.request.dshot_min
        while val < goal_handle.request.dshot_max:
            self.get_logger().info(f"fuck you {goal_handle.request.dshot_max} {goal_handle.request.dshot_min} {goal_handle.request.step_size} {val}")
            measurementValues.append(round(val))
            measurementValuesNegative.append(-round(val))

            # step size is now a percent
            val = val * (100 + goal_handle.request.step_size) / 100.0

        measurementValues.append(goal_handle.request.dshot_max)
        measurementValuesNegative.append(-goal_handle.request.dshot_max)


        for value in measurementValuesNegative:
            measurementValues.append(value)

        # also append the last value
        for dshot_value in measurementValues:
            # Once negative dshot commands have finished, operator needs to flip thruster before continueing
            if dshot_value < 0 and flag:
                flag = False
                flip_force = -1

                # stop while waiting
                self.dshot = 0
                self.publish_dshot_command()
                self.feedback_message(goal_handle, 0,
                                      "Please flip thruster and publish command/trigger once complete")
                self.wait_for_input()

            # Send dshot command to thruster
            self.dshot = dshot_value
            self.publish_dshot_command()

            # Send action feedback on current status
            self.feedback_message(goal_handle, dshot_value, f"Current dshot value: {dshot_value}")

            # Delay for transients to settle and for thruster to respond
            time.sleep(DELAY_TIME)

            # Collect data and store it
            (avg_rpm, avg_current, avg_voltage,
             avg_force) = self.get_data(N_SAMPLES)
            rows_data.append(
                [dshot_value, avg_rpm, avg_current, avg_voltage, flip_force * avg_force])

            self.get_logger().info("Saving: " + str(dshot_value) + " Avg RPM: " +
                                   str(avg_rpm) + " Avg Force: " + str(avg_force) + " Avg current: " + str(avg_current) + " Avg voltage: " + str(avg_voltage))
            
            if goal_handle.is_cancel_requested:
                self.feedback_message(goal_handle, 0, "Acknowledged cancel request. Canceling...")
                self.dshot = NEUTRAL_DSHOT
                self.publish_dshot_command()
                goal_handle.canceled()
                result = CalibrateThruster.Result()
                return result
                

        # Data collection finished, send command to turn off thruster
        self.dshot = NEUTRAL_DSHOT
        self.publish_dshot_command()

        # Save data and report results
        result = CalibrateThruster.Result()
        self.save_data(rows_data, goal_handle.request.file)
        goal_handle.succeed()
        self.running = False
        return result

    def get_data(self, N_samples):
        # Method starts data collect, and returns average results after N_samples have been collected
        # Once collecting_data is True, the rpm and force lists will get appended by subscriber callbacks

        # Clear previous data and start collecting
        self.rpm = []
        self.current = []
        self.voltage = []
        self.force = []
        self.collecting_data = True

        # Wait for samples to collect, report error if the collection is taking too long
        start_time = time.time()
        while rclpy.ok() and min(len(self.rpm), len(self.force)) < N_samples:
            if time.time()-start_time > COLLECTION_TIMEOUT:
                self.get_logger().error(
                    f"Data collection timed out at dshot value {self.dshot}")
                # Data failed to collect, return imaginary numbers to indicate failure
                return (1j, 1j, 1j, 1j)
        # Stop data collection and return averages
        self.collecting_data = False
        return (mean(self.rpm), mean(self.current), mean(self.voltage), mean(self.force))

    def save_data(self, data, file_name) -> str:
        # Saves each row of data into a csv file and returns the file name
        try:
            with open(file_name, 'w', newline='') as csv_file:
                # Write data in file
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow(
                    ["dshot", "rpm", "current", "voltage", "force (N)"])
                csv_writer.writerows(data)

            # File saved successfully, return the file name
            return file_name
        except Exception as e:
            # Error opening file, report error
            self.get_logger().error(str(e))
            self.get_logger().info(
                f"File could not open, here's the data: {data}")
            return ""  # Return an empty string to indicate failure

    def get_thruster_length(self) -> int:
        # Returns amount of thrusters on vehicle
        # Load thruster info
        self.declare_parameter("vehicle_config", "")
        config_path = self.get_parameter("vehicle_config").value
        if (config_path == ''):
            self.get_logger().fatal(
                "vehicle config file param not set or empty, using assumed thruster count of 8")
            return 8
        with open(config_path, 'r') as stream:
            config_file = yaml.safe_load(stream)
        thruster_info = config_file['thrusters']
        return len(thruster_info)

    def wait_for_input(self):
        # Goes into loop until Node has recieved a trigger command
        self.triggered = False
        while rclpy.ok() and not self.triggered:
            pass

    #######################################
    #    Callback and timer functions     #
    #######################################

    def publish_dshot_command(self):
        # Publishes dshot value to specified thruster and NUETRAL_DSHOT to all other thrusters
        # Function is called on timer to prevent firmware from timing out
        dshot_msg = DshotCommand()
        dshot_values = [NEUTRAL_DSHOT] * self.thruster_length

        if (NEGATIVE):
            dshot_values[self.thruster_num] = -self.dshot
        else:
            dshot_values[self.thruster_num] = self.dshot

        dshot_msg.values = dshot_values
        self.dshot_pub.publish(dshot_msg)

    def trigger_callback(self, msg: Empty):
        # Indicated user has triggered to move on in code
        self.triggered = True

    def rpm_callback(self, msg: DshotRPMFeedback):
        # Adds rpm data to list if data is collecting
        if self.collecting_data:
            self.rpm.append(msg.rpm[self.thruster_num])

    def force_callback(self, msg: Float32):
        # Adds force data to list if data is collecting
        if self.collecting_data:
            self.force.append(msg.data)

    def electrical_callback(self, msg: BatteryStatus):

        # Ensure there is only one battery connected, if it reads two different serial number print error
        if self.battery_serial is None:
            self.battery_serial = msg.serial
        elif msg.serial != self.battery_serial:
            self.get_logger().fatal("ERROR: Only one battery should be connected")
            rclpy.shutdown()

        # Saves present current for getting idle
        self.present_current = msg.pack_current

        # Add voltage and current to data
        if self.collecting_data:
            self.current.append(
                (msg.pack_current - self.idle_current))
            self.voltage.append(msg.pack_voltage)

    ##########################################
    #     Boiler plate action functions      #
    ##########################################

    def goal_callback(self, goal_request):
        # Accept or reject a client request to begin an action
        if self.running:
            return GoalResponse.REJECT
        else:
            self.running = True
            return GoalResponse.ACCEPT

    def cancel_callback(self, goal):
        return CancelResponse.ACCEPT

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    thruster_cal_action_server = CalibrateThrusterAction()

    executor = MultiThreadedExecutor()
    rclpy.spin(thruster_cal_action_server, executor=executor)

    thruster_cal_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
