import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rb3302-5/rb3302/robot_ws/install/waverover_base'
