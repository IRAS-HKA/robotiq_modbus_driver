from pymodbus.client.sync import ModbusSerialClient
from math import ceil
import sys
import time


class GripperStatus(object):

    def __init__(self, init_status) -> None:

        self.init_status = init_status

        # ACT=1: activated; ACT=1: reset
        self.gACT = (init_status[0] >> 0) & 0x01

        # GTO=1: go to position request; GTO=0: Standby
        self.gGTO = (init_status[0] >> 3) & 0x01

        # STA=0: is in reset state; STA=1: Activation in progress;
        # STA=2: Not used; STA=3: Activation is completed
        self.gSTA = (init_status[0] >> 4) & 0x03

        # OBJ=0: Fingers are in motion (only meanful if gGTO=1)
        # OBJ=1: Fingers have stopped due to a contact while opening
        # OBJ=2: Fingers have stopped due to a contact while closing
        # OBJ=3: Fingers are at requested posision
        self.gOBJ = (init_status[0] >> 6) & 0x03

        # FLT=0x00: No Fault
        # FLT=0x05: Priority Fault: action delayed, initialization must be completed prior to action
        # FLT=0x07: Priority Fault: The activation bit must be set prior to action
        # FLT=0x09: Minor Fault > The communication chip is not ready (may be rebooting)
        # FLT=0x0B: Minor Fault > Automatic release in progress
        # FLT=0x0E: Major Fault > Overcurrent protection triggered
        # FLT=0x0F: Major Fault > Automatic release is completed
        self.gFLT = init_status[2]

        # requested position for the gripper x/255
        self.gPR = init_status[3]

        # Actual position of fingers: x/255
        self.gPO = init_status[4]

        # Actual current of fingers (gCU * 10) mA
        self.gCU = init_status[5]

    def update_status(self, status):

        self.gACT = (status[0] >> 0) & 0x01
        self.gGTO = (status[0] >> 3) & 0x01
        self.gSTA = (status[0] >> 4) & 0x03
        self.gOBJ = (status[0] >> 6) & 0x03
        self.gFLT = status[2]
        self.gPR = status[3]
        self.gPO = status[4]
        self.gCU = status[5]

    def print_info(self):
        output = "\n =========== Gripper Status in =========== \n"

        # gACT
        output += 'gACT = ' + str(self.gACT) + ': '
        if (self.gACT == 0):
            output += "Gripper reset \n"
        elif (self.gACT == 1):
            output += "Gripper activation\n"

        # gGTO
        output += 'gGTO = ' + str(self.gGTO) + ': '
        if (self.gGTO == 0):
            output += 'Standby (or performing activation/automatic release)\n'
        if (self.gGTO == 1):
            output += 'Go to Position Request\n'

        # gSTA
        output += 'gSTA = ' + str(self.gSTA) + ': '
        if (self.gSTA == 0):
            output += 'Gripper is in reset ( or automatic release ) state. see Fault Status if Gripper is activated\n'
        if (self.gSTA == 1):
            output += 'Activation in progress\n'
        if (self.gSTA == 2):
            output += 'Not used\n'
        if (self.gSTA == 3):
            output += 'Activation is completed\n'

        # gOBJ
        output += 'gOBJ = ' + str(self.gOBJ) + ': '
        if (self.gOBJ == 0):
            output += 'Fingers are in motion (only meaningful if gGTO = 1)\n'
        if (self.gOBJ == 1):
            output += 'Fingers have stopped due to a contact while opening\n'
        if (self.gOBJ == 2):
            output += 'Fingers have stopped due to a contact while closing \n'
        if (self.gOBJ == 3):
            output += 'Fingers are at requested position\n'

        # gFLT
        output += 'gFLT = ' + str(self.gFLT) + ': '
        if (self.gFLT == 0x00):
            output += 'No Fault\n'
        if (self.gFLT == 0x05):
            output += 'Priority Fault: Action delayed, initialization must be completed prior to action\n'
        if (self.gFLT == 0x07):
            output += 'Priority Fault: The activation bit must be set prior to action\n'
        if (self.gFLT == 0x09):
            output += 'Minor Fault: The communication chip is not ready (may be booting)\n'
        if (self.gFLT == 0x0B):
            output += 'Minor Fault: Automatic release in progress\n'
        if (self.gFLT == 0x0E):
            output += 'Major Fault: Overcurrent protection triggered\n'
        if (self.gFLT == 0x0F):
            output += 'Major Fault: Automatic release completed\n'

        # gPR
        output += 'gPR = ' + str(self.gPR) + ': '
        output += 'Echo of the requested position for the Gripper: ' + str(self.gPR) + '/255\n'

        # gPO
        output += 'gPO = ' + str(self.gPO) + ': '
        output += 'Position of Fingers: ' + str(self.gPO) + '/255\n'

        # gCU
        output += 'gCU = ' + str(self.gCU) + ': '
        output += 'Current of Fingers: ' + str(self.gCU * 10) + ' mA\n'

        print(output)


class GripperCommand(object):

    def __init__(self) -> None:
        self.reset_command()

    def reset_command(self):
        self.rACT = 1  # 0: reset
        self.rGTO = 0  # 1: go to position request
        self.rATR = 0  #

        self.rPR = 0  # position 0 - 255
        self.rSP = 0  # speed 0 - 255
        self.rFR = 50  # force 0 - 255

    def get_command_rtu_msg(self, action=None, position=None):
        """
        Generate message transfered to gripper via Modbus RTU
        Args:
            - action: 'activate', 'reset', 'open', 'close'
        Returns:
            - msg: list of values for registery
        """
        if action:
            self.update_command(action, position=position)
        msg = []
        msg.append(self.rACT + (self.rGTO << 3) + (self.rATR << 4))
        msg.append(0)
        msg.append(0)
        msg.append(self.rPR)
        msg.append(self.rSP)
        msg.append(self.rFR)

        return msg

    def update_command(self, action, position=None, speed=None, force=None):
        # activate
        if action == 'activate':
            self.reset_command()
            self.rACT = 1
            self.rGTO = 1
            self.rSP = 255
            self.rFR = 50

        # reset
        elif action == 'reset':
            self.reset_command()
            self.rACT = 0

        # close
        elif action == 'close':
            self.rPR = 255

        # open
        elif action == 'open':
            self.rPR = 0

        # go to position
        if position is not None:
            assert (position >= 0 & position <= 255), "The value of goal position should between 0 and 255"
            self.rPR = max(0, position)
            self.rPR = min(255, self.rPR)

        # desired speed
        if speed is not None:
            assert (speed >= 0 & speed <= 255), "The value of desired finger speed should between 0 and 255"
            self.rSP = max(0, speed)
            self.rSP = min(255, self.rSP)

        # desired force
        if force is not None:
            assert (force >= 0 & force <= 255), "The value of desired grasp force should between 0 and 255"
            self.rFR = max(0, force)
            self.rFR = min(255, self.rFR)

    def print_current_command(self, debug=True):

        output = "=========== Gripper Command in Loop =========== \n"
        output += "rACT = {}\n".format(self.rACT)
        output += "rGTO = {}\n".format(self.rGTO)
        output += "rATR = {}\n".format(self.rATR)
        output += "rPR  = {}\n".format(self.rPR)
        output += "rSP  = {}\n".format(self.rSP)
        output += "rFR  = {}\n".format(self.rFR)

        if debug:
            print(output)


class CommunicationModbusRtu:

    def __init__(self):
        self.client = None

    def connect_to_device(self, device):
        """
        Connection to the client - the method takes the tty_id (usually /dev/ttyUSB0) as an argument.
        """
        self.client = ModbusSerialClient(
            method='rtu', port=device, stopbits=1, bytesize=8, baudrate=115200, timeout=0.2)
        if not self.client.connect():
            print("[Error] Unable to connect to %s" % device)
            return False
        return True

    def disconnect_from_device(self):
        """
        Close connection
        """
        self.client.close()

    def write_device_register(self, data):
        """
        Send a command to the Gripper - the method takes a list of uint8 as an argument.
        The meaning of each variable depends on the Gripper model
        (see support.robotiq.com for more details)
        """
        # make sure data has an even number of elements
        if (len(data) % 2 == 1):
            data.append(0)

        # Initiate message as an empty list
        message = []
        # Fill message by combining two bytes in one register
        for i in range(0, int(len(data) / 2)):
            message.append((data[2 * i] << 8) + data[2 * i + 1])

        # To do!: Implement try/except
        self.client.write_registers(0x03E8, message, unit=0x0009)

    def read_device_register(self, num_bytes=6):
        """
        Sends a request to read, wait for the response and returns the Gripper status.
        The method gets the number of bytes to read as an argument
        Args:
            - num_bytes: length of registery to be read
        Returns:
            - output: list contains num_bytes data, by default 6.
        """

        num_regs = int(ceil(num_bytes / 2.0))

        # To do!: Implement try/except
        # Get status from the device
        response = self.client.read_holding_registers(
            0x07D0, num_regs, unit=0x0009)
        print(type(response))
        # Instantiate output as an empty list
        output = []

        # Fill the output with the bytes in the appropriate order
        for i in range(0, num_regs):
            output.append((response.getRegister(i) & 0xFF00) >> 8)
            output.append(response.getRegister(i) & 0x00FF)

        return output


class RobotiqGripperModbusDriver(CommunicationModbusRtu):

    def __init__(self, device_id):
        self.device_id = device_id
        self.gripper_status = None
        self.gripper_command = GripperCommand()
        super().__init__()

    def connect(self):
        self.connect_to_device(device=self.device_id)
        # get initial gripper status
        init_status = self.read_device_register()
        self.gripper_status = GripperStatus(init_status=init_status)
        self.gripper_status.print_info()
        # activate
        self.activate()

    def get_status(self):
        status = self.read_device_register()
        self.gripper_status.update_status(status)
        self.gripper_status.print_info()

    # actions
    def activate(self):
        msg = self.gripper_command.get_command_rtu_msg(action='activate')
        self.write_device_register(msg)

    def reset(self):
        msg = self.gripper_command.get_command_rtu_msg(action='reset')
        self.write_device_register(msg)

    def open(self):
        msg = self.gripper_command.get_command_rtu_msg(action='open')
        self.write_device_register(msg)

    def close(self):
        msg = self.gripper_command.get_command_rtu_msg(action='close')
        self.write_device_register(msg)

    def go_to_position(self, position):
        msg = self.gripper_command.get_command_rtu_msg(action='close', position=position)
        self.write_device_register(msg)

    # for testing purpose
    def interactive_commanding(self):
        prompt = "==== Available Commands =====\nA: Activate \nR: Reset \nC: Close \nO: Open \n(0-255): Goal position \n>>>"
        command = input(prompt)
        if command in ['A', 'a']:
            self.activate()
        elif command in ['R', 'r']:
            self.reset()
        elif command in ['C', 'c']:
            self.close()
        elif command in ['O', 'o']:
            self.open()
        else:
            try:
                goal_position = int(command)
                # print("Desired goal position: ", goal_position)
                self.go_to_position(goal_position)
            except ValueError:
                print("Unknown command: {}".format(command))

    def kill(self):
        self.disconnect_from_device()
        print("Disconnected to device")
        exit(0)


if __name__ == '__main__':

    try:
        dev_id = sys.argv[1]
        print('Get device ID: {}'.format(dev_id))
    except:
        dev_id = '/dev/ttyUSB0'
        print('Using default device ID: {}'.format(dev_id))

    gripper = RobotiqGripperModbusDriver(device_id=dev_id)
    gripper.connect()
    gripper.get_status()

    try:
        while True:
            time.sleep(0.2)
            gripper.interactive_commanding()
            gripper.get_status()
    except KeyboardInterrupt:
        pass
