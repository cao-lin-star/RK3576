"""CI1302 fixed-frame transport. No ROS or motion operations in the worker."""
import json
import os
import queue
import threading
import time
from collections import deque


class Protocol:
    def __init__(self, path):
        with open(path, encoding='utf-8') as stream:
            entries = json.load(stream)['entries']
        self.speech = {bytes.fromhex(r['rx']): r['speech'] for r in entries if r['rx']}
        self.commands = {bytes.fromhex(r['tx']): r['word'] for r in entries if r['tx']}
        expected = {bytes.fromhex('BB 01 00 66'): '妮妮',
                    bytes.fromhex('BB 01 01 66'): '回到基站',
                    bytes.fromhex('BB 01 02 66'): '回到起点',
                    bytes.fromhex('BB 01 03 66'): '开始自动建图'}
        if self.commands != expected or len(self.speech) != 57:
            raise ValueError('voice workbook protocol mismatch')


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.awake_until = 0.
        self.last_byte = 0.
        self.last_command = -100.

    def feed(self, data, now):
        if now-self.last_byte > .25:
            self.buffer.clear()
        self.last_byte = now
        self.buffer.extend(data)
        result = []
        while len(self.buffer) >= 4:
            frame = bytes(self.buffer[:4])
            if frame[:2] != b'\xbb\x01' or frame[3] != 0x66 or frame[2] not in (0, 1, 2, 3):
                del self.buffer[0]
                continue
            del self.buffer[:4]
            if frame[2] == 0:
                self.awake_until = now+15.
            elif now < self.awake_until and now-self.last_command >= 2.:
                result.append({1: 'dock', 2: 'start', 3: 'mapping'}[frame[2]])
                self.last_command = now
                self.awake_until = 0.  # One movement request per wakeup.
        return result


class VoiceSerial:
    def __init__(self, device, protocol):
        self.device, self.protocol = device, protocol
        self.incoming = queue.Queue(maxsize=4)
        self.pending = deque(maxlen=8)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.connected = False
        self.error = ''
        self.decoder = Decoder()
        self.thread = threading.Thread(target=self._run, name='ci1302-serial', daemon=True)
        self.thread.start()

    def say(self, group, item, urgent=False):
        frame = bytes((0xaa, group, item, 0x55))
        if frame not in self.protocol.speech:
            raise ValueError('unknown speech frame')
        if not self.connected:
            return
        with self.lock:
            if any(p[0] == frame for p in self.pending):
                return
            if urgent:
                self.pending.clear()
            self.pending.append((frame, time.monotonic()+12.))

    def clear(self):
        with self.lock:
            self.pending.clear()

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=1.)

    def _run(self):
        # Linux only; imported here so offline protocol tests are portable.
        import fcntl
        import select
        import termios
        import tty
        while not self.stop_event.is_set():
            fd = None
            try:
                fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                fcntl.ioctl(fd, termios.TIOCEXCL)
                tty.setraw(fd)
                attrs = termios.tcgetattr(fd)
                attrs[4] = attrs[5] = termios.B9600
                attrs[2] = (attrs[2] & ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.CRTSCTS)) | termios.CS8 | termios.CLOCAL | termios.CREAD
                termios.tcsetattr(fd, termios.TCSANOW, attrs)
                termios.tcflush(fd, termios.TCIOFLUSH)
                self.decoder = Decoder()
                self.connected, self.error = True, ''
                next_tx = time.monotonic()+1.
                while not self.stop_event.is_set():
                    ready, _, _ = select.select([fd], [], [], .05)
                    now = time.monotonic()
                    if ready:
                        data = os.read(fd, 256)
                        if not data:
                            raise OSError('voice serial disconnected')
                        for command in self.decoder.feed(data, now):
                            try: self.incoming.put_nowait((command, now))
                            except queue.Full: pass
                    packet = None
                    with self.lock:
                        while self.pending and self.pending[0][1] < now:
                            self.pending.popleft()
                        if self.pending and now >= next_tx:
                            packet = self.pending.popleft()[0]
                    if packet:
                        # Do not replay partially transmitted commands after reconnect.
                        if os.write(fd, packet) != len(packet):
                            raise OSError('short voice write')
                        duration = max(2.5, len(self.protocol.speech[packet])*.34+1.)
                        next_tx = now+duration
            except (OSError, ValueError) as error:
                self.error = str(error)
            finally:
                self.connected = False
                self.clear()
                self.decoder = Decoder()
                while not self.incoming.empty():
                    try: self.incoming.get_nowait()
                    except queue.Empty: break
                if fd is not None:
                    os.close(fd)
            self.stop_event.wait(2.)
