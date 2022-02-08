"""
A simple test server that returns a random number when sent the text "temp" via Bluetooth serial.
"""
import threading
import enum

from bluetooth import *

class ConnectionState(enum.Enum):
    STARTUP = 0
    CONNECTED = 1
    DISCONNECTED = 2

class BluetoothServer:

    def __init__(self, recv_callback) -> None:
        self._state = ConnectionState.STARTUP
        self._socket = BluetoothSocket(RFCOMM)
        self._socket.bind(("", PORT_ANY))
        self._socket.listen(1)
        self.client_sock = None

        self.recv_callback = recv_callback

        uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"

        advertise_service(
            self._socket,
            "TestServer",
            service_id=uuid,
            service_classes=[uuid, SERIAL_PORT_CLASS],
            profiles=[SERIAL_PORT_PROFILE],
        )

    def run(self):
        t = threading.Thread(target=self._server_loop)
        t.start()
        print("Started...")
        
    def _server_loop(self):
        while True:
            if self._state == ConnectionState.STARTUP or self._state == ConnectionState.DISCONNECTED:
                # wait for connenction
                print("Waiting for connection on RFCOMM channel %d" % self._socket.getsockname()[1])
                self.client_sock, client_info = self._socket.accept()
                self._state = ConnectionState.CONNECTED
                print("Accepted connection from ", client_info)

            elif self._state == ConnectionState.CONNECTED:
                # handle connection
                try:
                    req = self.client_sock.recv(1024)
                    if len(req) == 0:
                        break

                    self.recv_callback(req)

                except IOError:
                    print("disconnected")
                    self._state = ConnectionState.DISCONNECTED

                except KeyboardInterrupt:
                    print("disconnected")
                    break
            else:
                raise RuntimeError("Unknown state")

        
if __name__ == "__main__":
    b = BluetoothServer(lambda x: print(x))
    b.run()

