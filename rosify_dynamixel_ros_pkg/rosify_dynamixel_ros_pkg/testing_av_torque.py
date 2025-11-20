from custom_dxl.CustomDXL import CustomDXL
import time

dxl = CustomDXL([1], baudrate=57600)
dxl.open_port()

print("Torque på, sender 1000...")
dxl.send_goal(0)
time.sleep(2)

#print("Sender 3000...")
#dxl.send_goal(0)
#time.sleep(2)

print("Leser posisjon...")
print(dxl.read_motor_positions())
