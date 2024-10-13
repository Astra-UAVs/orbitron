import logging
import socket
import threading
import json
import zlib
import lzma
import base64

class NodeConnection(threading.Thread):

	def __init__(self, main_node, sock, id, host, port):
		self.host = host
		self.port = port

		self.main_node = main_node

		self.sock = sock
		self.terminate_flag = threading.Event()

		self.id = str(id)

		self.EOT_CHAR = 0x04.to_bytes(1, 'big')
		self.COMPR_CHAR = 0x02.to_bytes(1, 'big')

		self.info = {}

		self.sock.settimeout(10.0)

		self.main_node.logger.debug(f"NodeConnection: Started with client ({self.id}) '{self.host}:{self.port}'")

		super(NodeConnection, self).__init__()

	def compress(self, data, compression):
		self.main_node.logger.debug(f"{self.id}: compress: {compression}")
		self.main_node.logger.debug(f"{self.id}: compress:input: {str(data)}")

		compressed = data

		try:
			if compression == 'zlib':
				compressed = base64.b64encode(zlib.compress(data, 6) + b'zlib')
			elif compression == 'lzma':
				compressed = base64.b64encode(lzma.compress(data) + b'lzma')
			else:
				self.main_node.logger.debug(f"{self.id}: compress:Unknown compression")
		except Exception as ex:
			self.main_node.logger.debug("compress: exception: " + str(e))

		return compressed

	def decompress(self, compressed):
		self.main_node.logger.debug(f"{self.id}:decompress:input: {str(compressed)}" )
		compressed = base64.b64decode(compressed)
		self.main_node.logger.debug(f"{self.id}:decompress:b64decode: {str(compressed)}")

		try:
			if compressed[-4:] == b'zlib':
				compressed = zlib.decompress(compressed[0:len(compressed) - 4])
			elif compression[-4:] == b'lzma':
				compressed = lzma.decompress(compressed[0:len(compressed) - 4])
		except Exception as e:
			self.main_node.logger.debug(f"Exception: {str(e)}")

		self.main_node.logger.debug(f"{self.id}:decompress:result: {str(compressed)}")
		return compressed

	def send(self, data, encoding_type='utf-8', compression='none'):
		if isinstance(data, str):
			try:
				if compression == 'none':
					self.sock.sendall(data.encode(encoding_type) + self.EOT_CHAR)
				else:
					data = self.compress(data.encode(encoding_type), compression)
					if data != None:
						self.sock.sendall(data + self.COMPR_CHAR + self.EOT_CHAR)
			except Exception as e:
				self.main_node.logger.debug(f"nodeconnection send: Error sending data to node: {str(e)}")
				self.stop()
		elif isinstance(data, dict):
			try:
				if compression == 'none':
					self.sock.sendall(json.dumps(data).encode(encoding_type) + self.EOT_CHAR)
				else:
					data = self.compress(json.dumps(data).encode(encoding_type), compression)
					if data != None:
						self.sock.sendall(data + self.COMPR_CHAR + self.EOT_CHAR)
			except TypeError as type_err:
				self.main_node.logger.debug('This dict is invalid')
				self.main_node.logger.debug(type_err)
			except Exception as e:
				self.main_node.logger.debug(f"nodeconnection send: Error sending data to node: {str(e)}")
				self.stop()

		elif isinstance(data, bytes):
			try:
				if compression == 'none':
					self.sock.sendall(data + self.EOT_CHAR)
				else:
					data = self.compress(data, compression)
					if data != None:
						self.sock.sendall(data + self.COMPR_CHAR + self.EOT_CHAR)

			except Exception as e:
				self.main_node.logger.debug(f"nodeconnection send: Error sending data to node: {str(e)}")
				self.stop()
		else:
			self.main_node.logger.debug("datatype used is not valid please use str, dict or byte string")

	def stop(self):
		self.terminate_flag.set()

	def parse_packet(self, packet):
		if packet.find(self.COMP_CHAR) == len(packet) - 1:
			packet = self.decompress(packet[0:-1])
		try:
			packet_decoded = packet.decode('utf-8')
			try:
				return json.loads(packet_decoded)
			except json.decoder.JSONDecodeError:
				return packet_decoded
		except UnicodeDecodeError:
			return packet

	def run(self):
		buffer = b''
		while not self.terminate_flag.is_set():
			chunk = b''
			try:
				chunk = self.sock.recv(4096)
			except socket.timeout:
				self.main_node.logger.debug("NodeConnection: timeout")
			except Exception as e:
				self.terminate_flag.set()
				self.main_node.logger.debug("Unexpected error")
				self.main_node.logger.debug(e)

			if chunk != b'':
				buffer += chunk
				eot_pos = buffer.find(self.EOT_CHAR)

				while eot_pos > 0:
					packet = buffer[:eot_pos]
					buffer = buffer[eot_pos + 1:]

					self.main_node.message_count_recv += 1
					self.main_node.node_message(self, self.parse_packet(packet))

					eot_pos = buffer.find(self.EOT_CHAR)

			time.sleep(0.01)

		self.sock.settimeout(None)
		self.sock.close()
		self.main_node.node_disconnected(self)
		self.main_node.logger.debug("NodeConnection: Stopped")

	def set_info(self, key, value):
		self.info[key] = value

	def get_info(self, key):
		return self.info[key]

