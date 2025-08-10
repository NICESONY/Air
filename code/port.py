# port_list.py
import serial.tools.list_ports

"""
포트 확인 할 때 사용하는 코드
"""

ports = serial.tools.list_ports.comports()
for p in ports:
    print(p.device)   # e.g. COM3, COM4 등
