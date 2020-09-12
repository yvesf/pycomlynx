# Copyright (c) 2012, Florian Wagner <florian@wagner-flo.net> See https://bitbucket.org/wagnerflo/pycoly.
# Copyright (c) 2018, Yves Fischer <yvesf+git@xapek.org>.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import struct

__all__ = [
    'hdlc',
    'Address', 'ProtocolError', 'ParseError', 'Message',
    'PingRequest', 'PingReply', 'GetNodeInformationRequest', 'GetNodeInformationReply', 'CanRequest', 'CanReply',
    'Parameters',
]


class hdlc:
    _fcs_init = 0xffff
    _fcs_table = (
        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
        0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
        0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
        0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
        0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
        0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
        0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
        0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
        0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
        0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
        0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
        0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
        0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
        0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
        0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
        0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
        0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
        0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
        0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
        0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
        0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
        0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
        0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
        0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
        0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
        0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
        0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
        0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
        0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
        0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
        0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
        0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
    )

    @staticmethod
    def _fcs(iterator, yield_bytes):
        fcs = hdlc._fcs_init
        for byte in iterator:
            fcs = (fcs >> 8) ^ hdlc._fcs_table[(fcs ^ byte) & 0xff]
            if yield_bytes:
                yield byte
        fcs = fcs ^ hdlc._fcs_init
        yield fcs & 0x00ff
        yield fcs >> 8

    @staticmethod
    def fcs(iterator):
        return tuple(hdlc._fcs(iterator, False))

    @staticmethod
    def append_fcs(iterator):
        return hdlc._fcs(iterator, True)

    control_escape = 0x7d
    flag_sequence = 0x7e
    broadcast = 0xff
    unnumbered_info = 0x03

    @staticmethod
    def byte_stuff(iterator, additional=()):
        stuff = (hdlc.control_escape, hdlc.flag_sequence) + additional
        for byte in iterator:
            if byte in stuff:
                yield 0x7d
                yield byte ^ 0x20
            else:
                yield byte

    @staticmethod
    def byte_unstuff(iterator, count):
        for i in range(count):
            byte = next(iterator)
            if byte == hdlc.control_escape:
                yield ord(next(iterator)) ^ 0x20
            else:
                yield byte


class ProtocolError(Exception):
    pass


class ParseError(Exception):
    pass


def _chain(*items):
    for item in items:
        try:
            for i in item:
                yield i
        except TypeError:
            yield item


class Address:
    def __init__(self, network, subnet, node):
        if not (0 <= network < 16 and 0 <= subnet < 16 and 0 <= node < 256):
            raise ProtocolError('Invalid address')
        self.network = network
        self.subnet = subnet
        self.node = node

    def as_bytes(self):
        return (self.network << 4 | self.subnet, self.node)

    def __repr__(self):
        return "({:02x}-{:02x}-{:02x})".format(self.network, self.subnet, self.node)

    @classmethod
    def from_bytes(cls, first, second):
        return cls(first >> 4, first & 0x0f, second)

    @classmethod
    def broadcast(cls, network=0xf, subnet=0xf):
        return cls(network, subnet, 0xff)


message_types = {}


class Message(object):
    source = property(lambda self: self._source)
    destination = property(lambda self: self._destination)

    def __init__(self, source, destination, data=()):
        self._source = source
        self._destination = destination
        self._data = data

    def __iter__(self):
        return _chain(
            hdlc.flag_sequence,
            hdlc.byte_stuff(
                hdlc.append_fcs(
                    _chain(
                        hdlc.broadcast, hdlc.unnumbered_info,
                        self._source.as_bytes(),
                        self._destination.as_bytes(),
                        len(self._data), self._type, self._data))),
            hdlc.flag_sequence)

    def __str__(self):
        return u'<{}.{} source={} destination={} type=0x{:02x} data="{}">'.format(
            type(self).__module__, type(self).__name__,
            self._source, self._destination, self._type,
            ' '.join(hex(byte) for byte in self))

    @classmethod
    def unpack(cls, data_bytes):
        iterator = iter(data_bytes)

        def read(count):
            return tuple(hdlc.byte_unstuff(iterator, count))

        if read(3) != (hdlc.flag_sequence, hdlc.broadcast,
                       hdlc.unnumbered_info):
            raise ParseError()

        source = read(2)
        destination = read(2)
        size, type = read(2)
        data = read(size)

        if (type >> 5) & 0x1 == 1:
            raise ProtocolError("Application Error")
        if (type >> 6) & 0x1 == 1:
            raise ProtocolError("Transmission Error")

        crc = hdlc.fcs(_chain(hdlc.broadcast, hdlc.unnumbered_info,
                              source, destination, size, type, data))
        if read(2) != crc:
            raise ParseError('crc does not match {}'.format(crc))

        if read(1) != (hdlc.flag_sequence,):
            raise ParseError()

        source = Address.from_bytes(*source)
        destination = Address.from_bytes(*destination)

        if type in message_types:
            return message_types[type].unpack_data(source, destination, data)
        else:
            raise ProtocolError("Unknown type: {}".format(type))

    @classmethod
    def unpack_data(cls, source, destination, data):
        raise NotImplementedError()


class PingRequest(Message):
    _type = 0x15

    def __init__(self, source, destination):
        super(PingRequest, self).__init__(source, destination, ())

    @classmethod
    def unpack_data(cls, source, destination, data):
        return cls(source, destination)


message_types[PingRequest._type] = PingRequest


class PingReply(Message):
    _type = 0x95

    def __init__(self, source, destination):
        super(PingReply, self).__init__(source, destination, ())

    @classmethod
    def unpack_data(cls, source, destination, data):
        return cls(source, destination)


message_types[PingReply._type] = PingReply


class GetNodeInformationRequest(Message):
    _type = 0x13

    def __init__(self, source, destination):
        super(GetNodeInformationRequest, self).__init__(
            source, destination, 29 * (0xff,))

    @classmethod
    def unpack_data(cls, source, destination, data):
        return cls(source, destination)


message_types[GetNodeInformationRequest._type] = GetNodeInformationRequest


class GetNodeInformationReply(Message):
    _type = 0x93

    def __init__(self, source, destination, data):
        super(GetNodeInformationReply, self).__init__(
            source, destination, data)

    def get_product_number(self):
        return bytes(self._data[0:11]).decode('ascii')

    product_number = property(get_product_number)

    def get_serial_number(self):
        return bytes(self._data[12:23]).decode('ascii')

    serial_number = property(get_serial_number)

    def get_network_number(self):
        return self._data[24]

    network_number = property(get_network_number)

    def get_subnet_number(self):
        return self._data[25]

    subnet_number = property(get_subnet_number)

    def get_address(self):
        return self._data[26]

    address = property(get_address)

    def get_device_type_id(self):
        return self._data[27]

    device_type_id = property(get_device_type_id)  # "for future use"

    def get_device_type_sub_id(self):
        return self._data[28]

    device_type_sub_id = property(get_device_type_sub_id)  # "for future use"

    @classmethod
    def unpack_data(cls, source, destination, data):
        if len(data) != 0x1d:
            raise ParseError("Length of {} is not 29".format(cls))
        return cls(source, destination, data)


message_types[GetNodeInformationReply._type] = GetNodeInformationReply


class Parameters:
    class ULX:
        # Raw measured values
        RawInstantEnergyProduction1 = [0x01, 0x01, 0x01]
        RawInstantEnergyProduction2 = [0x01, 0x01, 0x02]
        RawInstantEnergyProduction3 = [0x01, 0x01, 0x03]
        RawInstantEnergyProduction4 = [0x01, 0x01, 0x04]
        TotalEnergyProduction1 = [0x01, 0x02, 0x01]
        TotalEnergyProduction2 = [0x01, 0x02, 0x02]
        TotalEnergyProduction3 = [0x01, 0x02, 0x03]
        TotalEnergyProduction4 = [0x01, 0x02, 0x04]
        # Smoothed measured values
        SmoothedInstantEnergyProduction = [0x02, 0x01, 0xd]
        GridVoltage = [0x02, 0x14, 0xd]
        GridCurrent = [0x02, 0x15, 0xd]
        GridFrequency = [0x02, 0x16, 0xd]
        PVVoltage1 = [0x02, 0x28, 0xd]
        PVVoltage2 = [0x02, 0x29, 0xd]
        PVVoltage3 = [0x02, 0x2a, 0xd]
        PVCurrent1 = [0x02, 0x2d, 0xd]
        PVCurrent2 = [0x02, 0x2e, 0xd]
        PVCurrent3 = [0x02, 0x2f, 0xd]
        # Status information
        LatestEvent = [0x0a, 0x28, 0x0d]
        OperationMode = [0x0a, 0x02, 0x04]
        ModulesInUseMask = [0x0a, 0x02, 0x04]
        LatestEventModule = [0x0a, 0x29, 0x0d]


class CanDataType:
    NOT_DEFINED = 0x0
    BOOLEAN = 0x1
    SIGNED_8 = 0x2
    SIGNED_16 = 0x3
    SIGNED_32 = 0x4
    UNSIGNED_8 = 0x5
    UNSIGNED_16 = 0x6
    UNSIGNED_32 = 0x7
    FLOAT = 0x8
    VISIBLE_STRING = 0x9
    PACKED_BYTES = 0xa
    PACKED_WORDS = 0xb

    @staticmethod
    def decode(type, value):
        if type == CanDataType.BOOLEAN:
            raise NotImplementedError("Boolean not implemented")
        elif type == CanDataType.UNSIGNED_8:
            return value[0]
        elif type == CanDataType.UNSIGNED_16:
            return struct.unpack('H', bytes(value[0:2]))[0]
        elif type == CanDataType.UNSIGNED_32:
            return struct.unpack('i', bytes(value[0:4]))[0]
        else:
            raise NotImplementedError('Data type not supported: {} for data: {}'.format(type, value))


class CanRequest(Message):
    """Embedded CAN Kingdom message"""
    _type = 0x01

    def __init__(self, source, destination, parameter_index, parameter_sub_index, destination_module):
        super(CanRequest, self).__init__(
            source, destination, 10 * [0x00, ])
        self._data[0] = 0xc8
        self._data[1] = destination_module  # Destination module id
        self._data[2] = 0xd0  #
        self._data[3] = parameter_index  # Parameter index
        self._data[4] = parameter_sub_index  # parameter sub-index
        self._data[5] = 0x80
        self._data[6] = 0x00
        self._data[7] = 0x00
        self._data[8] = 0x00

    @classmethod
    def unpack_data(cls, source, destination, data):
        msg = cls(source, destination)
        print(data)
        return msg


message_types[CanRequest._type] = CanRequest


class CanReply(Message):
    _type = 0x81

    def __init__(self, source, destination, data):
        super(CanReply, self).__init__(
            source, destination, data)

    def value(self):
        dt = self._data[5] & 0x0f
        return CanDataType.decode(dt, self._data[6:10])

    @classmethod
    def unpack_data(cls, source, destination, data):
        msg = cls(source, destination, data)
        return msg


message_types[CanReply._type] = CanReply
