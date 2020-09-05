import socket
import sys
import logging
import time

class TcpClient:
    def __init__(self, server_ip, server_port):
        self.non_rt_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_ip = server_ip
        self.server_port = server_port
        self.received_message = []
        self.received_message_str = ''
        self.logger = logging.getLogger('TcpClient')

    def connect_with_server(self, attempts = 5):
        # print client
        for attempt in range(attempts):
            try:
                self.non_rt_sock.connect((self.server_ip, self.server_port))
                self.logger.info('Connected to server {}'.format((self.server_ip, self.server_port)))
                break
            except socket.error as err:
                self.logger.warn('Connection attempt {}, Caught exception: {}'.format(attempt, err[1]))
                if attempt == attempts - 1:
                    self.logger.error('Failed to connect after {} attempts'.format(attempt))
                    return
                time.sleep(5)

    def send_msg_to_server(self, msg, attempts = 5):
        # print client
        for attempt in range(attempts):
            try:
                msg_len = len(msg)
                # msg_len_packed = struct.pack('I', msg_len)
                # header = '\xff'
                self.non_rt_sock.send(msg.encode())
                self.logger.info('Data sent to server')
                break
            except socket.error as error:
                # self.logger.error('Caught exception while sending: {}'.format(error[1]))
                time.sleep(1)

    def listen_for_payload(self):
        got_everything = False
        cumulative_buff_len = 0
        broken_packet = False
        self.payload = ''
        self.logger.debug('Listening for data')

        while got_everything == False:
            d = self.non_rt_sock.recvfrom(4096)
            data = d[0]
            addr = d[1]
            chunk_len = len(data)
            cumulative_buff_len += chunk_len
            # if broken_packet == False:
            print ("data:")
            print (data)
            self.received_message.append(data)
            self.received_message_str += self.received_message[-1]
            # payload_len = struct.unpack('h', data[1:3])[0]
            complete_msg = self.received_message_str.find("\n")

            if complete_msg != -1:
                got_everything = True
                print ("Received Msg: ")
                print (self.received_message_str)
                self.logger.debug('Message Received: {}'.format(self.received_message_str))
                self.received_message_str = ''
            else:
                self.logger.debug('Msg Chunk: {}'.format(self.received_message_str))

    def spin(self):
        frequency = 50.0
        spin_rate = 1 / frequency
        seq_id = 0
        current_velocity = 0.0
        max_speed = 3000 #rpm
        abs_tick = 0
        max_tick_count = 200000
        while True:
            # rt_interface = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            current_velocity = current_velocity + 0.5;
            abs_tick = abs_tick + 5
            seq_id = seq_id + 1
            if (current_velocity > max_speed):
                current_velocity = 0.0

            if (abs_tick > max_tick_count):
                abs_tick = 0

            left_wheel = '{seq_id=' + str(seq_id) + '&ts_seconds=120&ts_msec=448&axis_idx=1&axis_name=left_wheel&encoder_handle=2&current_velocity=' + str(current_velocity) +'&absolute_ticks='+ str(abs_tick) +'&digital_io='+ str(0) + '&}\n\r'
            right_wheel = '{seq_id=' + str(seq_id) + '&ts_seconds=120&ts_msec=448&axis_idx=2&axis_name=right_wheel&encoder_handle=2&current_velocity=' + str(current_velocity) +'&absolute_ticks='+ str(abs_tick) +'&digital_io='+ str(0) +'&}\n\r'

            # info_str = 'Hello How\'s life going\n'

            info_str = left_wheel + right_wheel;
            self.send_msg_to_server(info_str, 5)
            time.sleep(spin_rate)
            # self.listen_for_payload()

if __name__ == '__main__':
    logging.basicConfig(format='[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s',level=logging.DEBUG,filename='tcp_client.log')
    logging.debug('Logger created')
    tcp_client = TcpClient('localhost', 5100)
    tcp_client.connect_with_server()
    tcp_client.spin()
