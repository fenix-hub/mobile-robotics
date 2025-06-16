import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/fenixhub/ackermann_ws/install/ackermann_control_bridge'
