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

import selectors
import os
import logging
import termios
import tty
from comlynx import *


def execute(message):
    with SerialConnection('/dev/ttyUSB0') as conn:
        data = bytes(tuple(message))
        logging.debug("Write: %s", data)
        os.write(conn.fd, data)

        data = b''
        while conn.wait_for_data(1.0):
            data += os.read(conn.fd, 8)
            if len(data) > 1 and data.endswith(b'\x7e'):
                # one frame is complete then
                m = Message.unpack(data)
                logging.debug("Received: %s", m)
                yield m


class SerialConnection:
    def __init__(self, path):
        self._path = path
        self.fd = None
        self.sel = selectors.DefaultSelector()

    def __enter__(self):
        self.fd = os.open(self._path, os.O_RDWR | os.O_NOCTTY)

        tty.setraw(self.fd, tty.TCSAFLUSH)
        iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(self.fd)
        ispeed, ospeed = termios.B19200, termios.B19200
        cc[termios.VTIME] = 10
        cc[termios.VMIN] = 1
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])

        termios.tcflush(self.fd, termios.TCIOFLUSH)

        self.sel.register(self.fd, selectors.EVENT_READ, 'READ-READY')

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.sel.close()
        os.close(self.fd)

    def wait_for_data(self, timeout):
        for key, mask in self.sel.select(timeout=timeout):
            if key.data == 'READ-READY':
                return True
        return False

def main():
    source = Address(0x0, 0x0, 0x01)
    ping  =PingRequest(source, Address.broadcast())
    print(ping)
    for ping_reply in execute(ping):
        node_information: GetNodeInformationReply = next(execute(GetNodeInformationRequest(source, ping_reply.source)))
        device_name = "{}_{}".format(node_information.product_number, node_information.serial_number)

        for field, value in Parameters.ULX.__dict__.items():
            if field.startswith('_'):
                continue
            try:
                for message in execute(CanRequest(source, ping_reply.source, *value)):
                    if isinstance(message, CanReply):
                        print("comlynx.{}.{} value={}".format(device_name, field, message.value()))
                    else:
                        raise Exception(field)
            except ProtocolError as e:
                logging.error("%s %s", field, str(e))


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    main()
