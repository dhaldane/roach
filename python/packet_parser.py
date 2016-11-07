#!/usr/bin/python

import crcmod
import serial
import threading
import Queue
import struct
import time

crc8 = crcmod.mkCrcFun(0x1D5,initCrc=0,rev=False)

header_format = 'BBcBI'
header_fields = ['start','length','type','flags','sequence']

data_formats = {
  'C': {
    'format': 'iI',
    'fields': ['position_setpoint','current_setpoint']},
  'S': {
    'format': 'IiiHHHH',
    'fields': ['time','position','velocity','current','voltage','temperature','uc_temp']},
}

def parse_packet(packet):
  packet_type = packet[2]
  data_format = data_formats[packet_type]['format']
  data_fields = data_formats[packet_type]['fields']
  packet_format = header_format + data_format + 'B'
  packet_fields = header_fields + data_fields + ['crc']
  packet_values = struct.unpack(packet_format,packet)
  return dict(zip(packet_fields,packet_values))

def format_packet(packet):
  data_format = data_formats[packet['type']]['format']
  data_fields = data_formats[packet['type']]['fields']
  packet_format = header_format + data_format
  packet_fields = header_fields + data_fields
  packet.update({'start':0xAA, 'length':struct.calcsize(packet_format)+1})
  packet_values = [packet[field] for field in packet_fields]
  packet = struct.pack(packet_format, *packet_values)
  packet += chr(crc8(packet))
  return packet
  
class PacketParser(threading.Thread):
  def __init__(self, device='/dev/ttyUSB0',baud=1300000,stopbits=2):
    threading.Thread.__init__(self)
    self.daemon = True

    self.serial = serial.Serial(device,baudrate=baud)
    self.packet = ''
    self.queue = Queue.Queue()
    self.sequence = 0

  def __del__(self):
    self.flush()
    self.serial.close()

  def run(self):
    while(self.serial.isOpen()):
      self.packet = self.serial.read(1)
      if self.packet == '\xAA':
        self.packet += self.serial.read(1)
        length = ord(self.packet[-1])
        if length >= 9:
          self.packet += self.serial.read(length-2)
          #print ['%X' % ord(c) for c in self.packet]
          if crc8(self.packet) == 0:
            self.queue.put(self.packet)
  
  def get(self,block=False,timeout=None):
    try:
      return parse_packet(self.queue.get(block,timeout))
    except:
      return None

  def flush(self):
    rp = self.get()
    pkts = []
    while rp is not None:
      pkts.append(rp)
      rp = self.get()
    return pkts

  def put(self, packet):
    if type(packet) is dict:
      packet.update({'sequence':self.sequence})
      packet = format_packet(packet)

    if (self.serial.isOpen()):
      self.sequence += 1
      for c in packet:
        self.serial.write(c)
        time.sleep(0.0001)

ADCV = 3.4

def print_sensor_packet(packet):
  temp_v = (ADCV * packet['temperature']) / (2**16)
  # 500mV = 0C, 10mV/C
  temp = (temp_v - 0.4) / 0.0195
  #print 'Temp: %.2f C' % temp
  print 'P: %d, V:%f, @%d' % (packet['position'],packet['velocity']/(2.0**16),packet['time'])

def get_command_packet(p_setpoint, c_setpoint):
  return {
    'type': 'C',
    'flags': 0,
    'sequence': 0,
    'position_setpoint': p_setpoint,
    'current_setpoint': c_setpoint
  }

if __name__ == '__main__':
  pp = PacketParser()
  pp.start()
  while(True):
    time.sleep(0.001)
    packet = pp.get(False,timeout=0.01)
    if packet is not None:
      if packet['type'] == 'S':
        print_sensor_packet(packet)
      else:
        print packet
