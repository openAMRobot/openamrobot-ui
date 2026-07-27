const API_BASE = window.location.port === "3000" ? "http://127.0.0.1:5050" : "";

export const fetchSerialPorts = async () => {
  const response = await fetch(`${API_BASE}/api/devices/serial-ports`);
  if (!response.ok) throw new Error("Could not read serial ports from the backend");
  return response.json();
};
