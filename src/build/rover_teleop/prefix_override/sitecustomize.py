import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/deimos/deimosExNihilo_ws/src/install/rover_teleop'
