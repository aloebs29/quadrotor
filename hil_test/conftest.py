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


def pytest_addoption(parser):
    parser.addoption("--skip_flash", action="store_true", help="do not flash and reset the device under test")


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
                elif self.flash_done_event.is_set():
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


@pytest.fixture(autouse=True, scope="session")
def probe_rs_run(pytestconfig):
    command_str = "cargo make run"
    stop_event = threading.Event()
    flash_done_event = threading.Event()
    exc_queue = queue.Queue()

    probe_rs_thread = _ProbeRsThread(command_str, stop_event, flash_done_event, exc_queue)

    def stop_thread():
        stop_event.set()
        probe_rs_thread.join()
            
    # If the user requested that the DUT not be flashed/reset (i.e. it is already running), skip this.
    if not pytestconfig.getoption("skip_flash"):
        probe_rs_thread.start()

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
    
    if not pytestconfig.getoption("skip_flash"):
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
