import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rb3302-5/rb3302/student_ws/install/rb3302_nav'
