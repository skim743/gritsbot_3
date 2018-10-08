import gritsbot.gritsbotserial as gritsbotserial
import json
import vizier.node as node
import time
import argparse
from uuid import getnode
import logging
import queue

# Set up logging
global logger
logging.basicConfig(format='%(asctime)s - %(levelname)s - %(module)s - %(message)s')
logger = logging.getLogger('root')
logger.setLevel(logging.DEBUG)

# Constants
MAX_QUEUE_SIZE = 100


def get_mac():
    """Gets the MAC address for the robot from the network config info.

    Returns:
        A MAC address for the robot.

    """
    hex_mac = hex(getnode())[2:].zfill(12)
    return ':'.join(x + y for x, y in zip(hex_mac[::2], hex_mac[1::2]))


def create_node_descriptor(end_point):
    """Returns a node descriptor for the robot based on the end_point.

    The server_alive link is for the robot to check the MQTT connection periodically.

    Args:
        end_point (str): The ID of the robot.

    Returns:
        A node descriptor of the vizier format for the robot.

    """

    node_descriptor = \
        {
            'end_point': end_point,
            'links':
            {
                '/status': {'type': 'DATA'},
            },
            'requests':
            [
                {
                    'link': 'matlab_api/'+end_point,
                    'type': 'STREAM',
                    'required': False
                },
            ]
        }

    return node_descriptor

# Proposed request packet structure
# Read battery voltage
# request = {'request': 'read', 'iface': 'batt_volt'}

# Read charge status
# request = {'request': 'read', 'iface': 'charge_status'}

# Write motor velocities
# request = {'request': 'write', 'iface': 'motor', 'body': {'v': 0, 'w': 0}}

# Write left led
# request = {'request': 'write', 'iface': 'left_led', 'body': {'rgb': [255, 0, 0]}}

# Write right led
# request = {'request': 'write', 'iface': 'right_led', 'body': {'rgb': [255, 0, 0]}}

# Responses
# Battery voltage response
# response = {'status': 1, 'body': {'bat_volt': 4.3}}


class Request:

    def __init__(self, iface=[], request=[], body={}):
        self.iface = []
        self.request = []
        self.body = []

    def add_write_request(self, iface, body):
        self.iface.append(iface)
        self.request.append('write')
        self.body.append(body)

        return self

    def add_read_request(self, iface):
        self.iface.append(iface)
        self.request.append('read')
        self.body.append({})

        return self

    def to_json_encodable(self):
        req = {'request': self.request, 'iface': self.iface}

        if(self.body):
            req['body'] = self.body

        return req


def handle_write_response(status, body):
    return {}


def handle_read_response(iface, status, body):

    if(iface in body):
        return {iface: body[iface]}
    else:
        logger.critical('Request for ({0}) not in body ({1}) after request.'.format(iface, body))
        return {}


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("mac_list", help="JSON file containing MAC to id mapping")
    parser.add_argument("-port", type=int, help="MQTT Port", default=8080)
    parser.add_argument("-host", help="MQTT Host IP", default="localhost")
    parser.add_argument('-update_rate', type=float, help='Update rate for robot main loop', default=0.016)
    parser.add_argument('-status_update_rate', type=float, help='How often to check status info', default=1)

    # Retrieve the MAC address for the robot
    mac_address = get_mac()

    # Parser and set CLI arguments
    args = parser.parse_args()
    update_rate = args.update_rate
    status_update_rate = args.status_update_rate

    # Retrieve the MAC list file, containing a mapping from MAC address to robot ID
    try:
        f = open(args.mac_list, 'r')
        mac_list = json.load(f)
    except Exception as e:
        print(repr(e))
        print('Could not open file ({})'.format(args.node_descriptor))

    if(mac_address in mac_list):
        robot_id = mac_list[mac_address]
    else:
        print('MAC address {} not in supplied MAC list file'.format(mac_address))
        raise ValueError()

    logger.info('This is robot: ({0}) with MAC address: ({1})'.format(robot_id, mac_address))

    # Create node descriptor for robot and set up links
    node_descriptor = create_node_descriptor(mac_list[mac_address])
    status_link = robot_id + '/status'
    input_link = 'matlab_api/' + robot_id

    started = False
    robot_node = None
    while (not started):
        robot_node = node.Node(args.host, args.port, node_descriptor)
        try:
            robot_node.start()
            started = True
        except Exception as e:
            logger.critical('Could not start robot node.')
            logger.critical(repr(e))
            robot_node.stop()

        # Don't try to make nodes too quickly
        time.sleep(1)

    logger.info('Started robot node.')

    started = False
    serial = None
    while (not started):
        serial = gritsbotserial.GritsbotSerial(serial_dev='/dev/ttyACM0', baud_rate=500000)
        try:
            serial.start()
            started = True
        except Exception as e:
            # This class stops itself if the device cannot be initially acquired, so we don't need to stop it.
            logger.critical('Could not acquire serial device.')
            logger.critical(repr(e))

        # Don't try to acquire the serial device too quickly
        time.sleep(1)

    logger.info('Acquired serial device.')

    # Queues for STREAM links
    inputs = robot_node.subscribe(input_link)

    # Initialize times for various activities
    start_time = time.time()
    print_time = time.time()
    status_update_time = time.time()

    # Initialize data
    status_data = {'batt_volt': -1, 'charge_status': False}
    last_input_msg = {}

    # Main loop for the robot
    while True:
        start_time = time.time()

        # Serial requests
        request = Request()
        handlers = []

        # Retrieve status data: battery voltage and charging status
        if((start_time - status_update_time) >= status_update_rate):
            request.add_read_request('batt_volt').add_read_request('charge_status')
            handlers.append(lambda status, body: handle_read_response('batt_volt', status, body))
            handlers.append(lambda status, body: handle_read_response('charge_status', status, body))

            status_update_time = start_time

        # Process input commands
        input_msg = None
        # Make sure that the queue has few enough messages
        if(inputs.qsize() > MAX_QUEUE_SIZE):
            logger.critical('Queue of motor messages is too large.')

        try:
            # Clear out the queue
            while True:
                input_msg = inputs.get_nowait()
        except queue.Empty:
            pass

        if(input_msg is not None):
            try:
                input_msg = json.loads(input_msg.decode(encoding='UTF-8'))
            except Exception as e:
                logger.warning('Got malformed JSON motor message ({})'.format(input_msg))
                logger.warning(e)
                # Set this to None for the next checks
                input_msg = None

        # If we got a valid JSON input msg, look for appropriate commands
        if(input_msg is not None):
            last_input_msg = input_msg
            if('v' in input_msg and 'w' in input_msg):
                # Handle response?
                request.add_write_request('motor', {'v': input_msg['v'], 'w': input_msg['w']})
                handlers.append(handle_write_response)

            if('left_led' in input_msg):
                request.add_write_request('left_led', {'rgb': input_msg['left_led']})
                handlers.append(handle_write_response)

            if('right_led' in input_msg):
                request.add_write_request('right_led', {'rgb': input_msg['right_led']})
                handlers.append(handle_write_response)

        # Write to serial port
        response = None
        if(len(handlers) > 0):
            try:
                response = serial.serial_request(request.to_json_encodable())
            except Exception as e:
                logger.critical('Serial exception.')
                logger.critical(e)

        # Call handlers
        # We'll have a status and body for each request
        if(response is not None and 'status' in response and 'body' in response
           and len(response['status']) == len(handlers) and len(response['body']) == len(handlers)):
            status = response['status']
            body = response['body']
            # Ensure the appropriate handler gets each response
            for i, handler in enumerate(handlers):
                status_data.update(handler(status[i], body[i]))
        else:
            # If we should have responses, but we don't
            if(len(handlers) > 0):
                logger.critical('Malformed response ({})'.format(response))

        robot_node.put(status_link, status_data)

        # Print out status data
        if((start_time - print_time) >= status_update_rate):
            logger.info('Status data ({})'.format(status_data))
            logger.info('Last input message received ({})'.format(last_input_msg))
            print_time = time.time()

        logger.info(time.time() - start_time)

        # Sleep for whatever time is left at the end of the loop
        time.sleep(max(0, update_rate - (time.time() - start_time)))


if __name__ == '__main__':
    main()