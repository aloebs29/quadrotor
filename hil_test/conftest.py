import asyncio
import queue
import shlex
import threading
import time

import bleak
import pytest
import pytest_asyncio
import serial
from serial.tools import list_ports

FLASH_TIMEOUT = 20.0
RESET_TIME = 2.0
SERIAL_TIMEOUT = 2.0

class _ProbeRsThread(threading.Thread):
    def __init__(self, command_str: list, stop_event: threading.Event, flash_done_event: threading.Event, exc_queue: queue.Queue, *args, **kwargs):
        super(_ProbeRsThread, self).__init__(*args, **kwargs)
        self.command = shlex.split(command_str)
        self.stop_event = stop_event
        self.flash_done_event = flash_done_event
        self.exc_queue = exc_queue

    async def run_command(self):
        proc = await asyncio.create_subprocess_exec(*self.command, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
        while not self.stop_event.is_set():
            # Check if the process has terminated
            ret = proc.returncode
            if ret is not None:
                raise IOError(f"Probe-rs process terminated early (returncode {ret}).")

            # Try to read a line
            try:
                line = await asyncio.wait_for(proc.stdout.readline(), timeout=0.1)
            except asyncio.TimeoutError:
                continue
            else:
                line = line.decode()
                if line.find("Finished in") != -1 and not self.flash_done_event.is_set():
                    self.flash_done_event.set()
                print(line.rstrip())
        
        proc.terminate()
        # NOTE: The asyncio.subprocess docs say not to use wait() if you're piping stdout/stderr, and to use
        #       communicate() instead. However, communicate() does not appear to block until the process finishes.
        await proc.wait()
            
    def run(self):
        # NOTE: We need to use asyncio here to be able to read lines from probe-rs in a non-blocking manner
        try:
            asyncio.run(self.run_command())
        except BaseException as e:
            self.exc_queue.put(e)


# NOTE: Enable autouse here to rebuild/flash/run on every test invocation. This should really be a dependency of all
#       other fixtures, but its convenient to leave the device running (without reflash/reset) while developing tests,
#       and autouse makes this a lot easier to toggle. Pytest seems to run this fixture first when `autouse=True`.
@pytest.fixture(autouse=False, scope="session")
def probe_rs_run():
    command_str = "cargo make run"
    stop_event = threading.Event()
    flash_done_event = threading.Event()
    exc_queue = queue.Queue()

    probe_rs_thread = _ProbeRsThread(command_str, stop_event, flash_done_event, exc_queue)
    probe_rs_thread.start()

    def stop_thread():
        stop_event.set()
        probe_rs_thread.join()

    # Wait for flashing to complete
    start = time.perf_counter()
    while not flash_done_event.is_set():
        try:
            # Check for timeout
            if time.perf_counter() - start > FLASH_TIMEOUT:
                raise TimeoutError
            
            # Check for exception raised by child thread
            try:
                exc = exc_queue.get_nowait()
            except queue.Empty:
                pass
            else:
                raise exc
    
        except BaseException as exc:
            stop_thread()
            raise exc

    time.sleep(RESET_TIME)
    yield
    stop_thread()


@pytest.fixture(scope="session")
def serial_usb():
    dev = None
    for port in list_ports.comports():
        if port.vid == 0xFEED and port.pid == 0xFACE:
            dev = port.device
            break
    else:
        pytest.fail("Did not find USB device.")
        
    ser = serial.Serial(dev, baudrate=115200, timeout=SERIAL_TIMEOUT)
    yield ser
    ser.close()


@pytest.fixture(scope="session")
def ble_address(serial_usb):
    serial_usb.write(b"mac_address\n")

    response = serial_usb.readline()
    address = bytes.decode(response).strip()
    return address


@pytest_asyncio.fixture(scope="session")
async def ble_client(ble_address):
    async with bleak.BleakClient(ble_address) as client:
        yield client
