/**
 * Connection types selectable when registering a device on the Devices
 * page (web/src/pages/DevicesPage.jsx). A contributor adding support for a
 * new connection type calls registerDeviceType() — DevicesPage.jsx itself
 * never needs editing.
 */
export const CONNECTION_TYPES = [
  { value: "usb", label: "USB", placeholder: "/dev/ttyUSB0" },
  { value: "can", label: "CAN bus", placeholder: "can0" },
  { value: "network", label: "Network", placeholder: "192.168.1.50:11411" },
  {
    value: "raspberry-pi",
    label: "Raspberry Pi (attached)",
    placeholder: "hostname or IP address",
  },
  { value: "other", label: "Other", placeholder: "connection target" },
];

export function registerDeviceType(entry) {
  CONNECTION_TYPES.push(entry);
}
