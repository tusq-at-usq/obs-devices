import pyudev
from typing import Optional

def show_current_vidpid() -> None:
    """Show VID:PID of connected USB devices."""
    context = pyudev.Context()
    for device in context.list_devices(subsystem="tty"):
        parent = device.find_parent("usb", "usb_device")
        if parent is None:
            continue
        if parent.get("ID_VENDOR_ID") and parent.get("ID_MODEL_ID"):
            print(
                f"Device: {device.device_node}, VID:PID = {parent.get('ID_VENDOR_ID')}:{parent.get('ID_MODEL_ID')}"
            )

def port_by_vidpid(vidpid: str) -> Optional[str]:
    """Return /dev/tty* port for given USB VID:PID."""
    context = pyudev.Context()
    for device in context.list_devices(subsystem="tty"):
        parent = device.find_parent("usb", "usb_device")
        if parent is None:
            continue
        if parent.get("ID_VENDOR_ID") and parent.get("ID_MODEL_ID"):
            if f"{parent.get('ID_VENDOR_ID')}:{parent.get('ID_MODEL_ID')}" == vidpid:
                return device.device_node.split("/")[-1]
    return None

if __name__ == "__main__":
    show_current_vidpid()
