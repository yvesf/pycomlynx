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

if __name__ == "__main__":
    import typing
    import asyncio

    import serial_asyncio
    import serial

    from comlynx import *


    @asyncio.coroutine
    def open_serial() -> typing.Tuple[asyncio.StreamReader, asyncio.StreamWriter]:
        reader = asyncio.StreamReader()
        ser = serial.serial_for_url('/dev/ttyUSB0', baudrate=19200)
        protocol = asyncio.StreamReaderProtocol(reader, loop=loop)
        transport = serial_asyncio.SerialTransport(loop, protocol, ser)
        writer = asyncio.StreamWriter(transport, protocol, reader, loop)
        return reader, writer


    @asyncio.coroutine
    def request(reader: asyncio.StreamReader, writer: asyncio.StreamWriter, message):
        data = bytes(tuple(message))
        print("Write: {}".format(data))
        writer.write(data)

        begin = yield from reader.read(1)
        if begin != bytes([hdlc.flag_sequence]):
            raise Exception('{}'.format(begin))

        data = begin + (yield from reader.readuntil(b'\x7e'))
        print("Read: {}".format(data))
        m = Message.unpack(iter(data))
        return m


    @asyncio.coroutine
    def ping():
        reader, writer = yield from open_serial()
        message = PingRequest(Address(0, 0, 2), Address.broadcast())
        response = yield from request(reader, writer, message)
        print(response)


    @asyncio.coroutine
    def get_node_request():
        reader, writer = yield from open_serial()
        message = GetNodeInformationRequest(Address(0x00, 0x00, 0x02), Address(0x01, 0x01, 0x3d))
        response = yield from request(reader, writer, message)
        print(response)


    @asyncio.coroutine
    def can():
        reader, writer = yield from open_serial()
        message = CanRequest(Address(0x00, 0x00, 0x02), Address(0x01, 0x01, 0x3d), *Parameters.ULX.LatestEvent)
        response = yield from request(reader, writer, message)
        print(response)


    loop = asyncio.get_event_loop()
    loop.run_until_complete(can())

    print("Close loop")
    loop.close()
