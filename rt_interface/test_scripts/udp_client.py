import socket
import sys
import logging
import time
import traceback
import struct

class UDPClient:
    def __init__(self, server_ip, server_port):
        self.non_rt_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_ip = server_ip
        self.server_port = server_port
        self.received_message = []
        self.received_message_str = ''
        self.logger = logging.getLogger('UdpClient')

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
                #self.non_rt_sock.send(msg.encode("utf-16"))
                self.non_rt_sock.send(msg)
                self.logger.info('Data sent to server')
                print ("Sent Msg! ")
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
            parsed_pkt = struct.unpack('BBB', data)
            print(parsed_pkt)
            self.received_message.append(data)
            # self.received_message_str += self.received_message[-1]
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
        while True:
            # rt_interface = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            info_str = 'Hello I am alive now. How\'s life going??\n'
            byte_str= struct.pack('ccc',b'\xFF',b'\x01',b'\x10')
            #byte_str= struct.pack('ccc',b'\xFF',b'\x01',b'\xDD')
            #byte_str= struct.pack('cccc',b'\xFF',b'\x02',b'\xCC',b'\xD0')
            #byte_str= struct.pack('cc',b'\xFF',b'\x02')
            #byte_str= struct.pack('c',b'\xCC')
            #byte_str= struct.pack('c',b'\xD0')
            #byte_str= struct.pack('cc',b'\xFF',b'\x01')
            #byte_str= struct.pack('c',b'\x10')
            #byte_str= struct.pack('c',b'\xFF')
            #String byte_string = new String(byte_str)
            self.send_msg_to_server(byte_str, 5)
            self.listen_for_payload()
    def read(self):
        d = self.non_rt_sock.recvfrom(4096)
        return d

if __name__ == '__main__':
    logging.basicConfig(format='[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s',level=logging.DEBUG,filename='udp_client.log')
    logging.debug('Logger created')
    tcp_client = UDPClient('localhost', 6000)
    tcp_client.connect_with_server()
    tcp_client.spin()
