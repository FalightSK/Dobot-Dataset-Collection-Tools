import threading
import socket
from time import sleep

class DobotApi:
    # Create a TCP client to a Dobot controller and manage synchronized send/recv.
    def __init__(self, ip, port, *args):
        # Store connection info.
        self.ip = ip
        self.port = port
        # Socket handle (0 means not connected).
        self.socket_dobot = 0
        # Lock to ensure send/receive pairs are not interleaved by multiple threads.
        self.__globalLock = threading.Lock()
        # Optional logging flag passed in args.
        if args:
            self.text_log = args[0]

        # Only known Dobot ports are accepted; otherwise warn.
        if self.port == 29999 or self.port == 30004 or self.port == 30005:
            try:
                # Create socket, connect, and enlarge receive buffer.
                self.socket_dobot = socket.socket()
                self.socket_dobot.connect((self.ip, self.port))
                self.socket_dobot.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 144000)
            except socket.error:
                # Print error if connection fails.
                print(socket.error)
        else:
            print(f"Connect to dashboard server need use port {self.port} !")

    # Log only when enabled by caller.
    def log(self, text):
        if self.text_log:
            print(text)

    # Send raw command text; if it fails, reconnect and retry until it succeeds.
    def send_data(self, string):
        try:
            self.socket_dobot.send(str.encode(string, 'utf-8'))
        except Exception as e:
            print(e)
            while True:
                try:
                    # Reconnect and resend the data.
                    self.socket_dobot = self.reConnect(self.ip, self.port)
                    self.socket_dobot.send(str.encode(string, 'utf-8'))
                    break
                except Exception:
                    # Wait before the next reconnect attempt.
                    sleep(1)

    # Read a response from the socket; reconnect on failure.
    def wait_reply(self):
        """
        Read the return value
        """
        data = ""
        try:
            # Receive up to 1024 bytes from Dobot.
            data = self.socket_dobot.recv(1024)
        except Exception as e:
            print(e)
            # If receive fails, reconnect for future calls.
            self.socket_dobot = self.reConnect(self.ip, self.port)

        finally:
            # Convert bytes to string; keep empty responses as-is.
            if len(data) == 0:
                data_str = data
            else:
                data_str = str(data, encoding="utf-8")
            return data_str

    # Cleanly close the socket connection.
    def close(self):
        """
        Close the port
        """
        if (self.socket_dobot != 0):
            try:
                self.socket_dobot.shutdown(socket.SHUT_RDWR)
                self.socket_dobot.close()
            except socket.error as e:
                print(f"Error while closing socket: {e}")

    # Send a command and wait for its response, serialized by a lock.
    def sendRecvMsg(self, string):
        """
        send-recv Sync
        """
        with self.__globalLock:
            # print("send:", string)
            self.send_data(string)
            recvData = self.wait_reply()
            return recvData

    # Ensure the socket is closed when the object is garbage-collected.
    def __del__(self):
        self.close()

    # Keep trying to reconnect until successful, then return the new socket.
    def reConnect(self, ip, port):
        while True:
            try:
                socket_dobot = socket.socket()
                socket_dobot.connect((ip, port))
                break
            except Exception:
                sleep(1)
        return socket_dobot