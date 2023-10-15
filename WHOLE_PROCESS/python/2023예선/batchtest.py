import subprocess
import atexit
from scripts.core import *
import matplotlib.pyplot as plt
import datetime
import matplotlib as mpl
import matplotlib.style as mplstyle
mpl.use('TkAgg')
mpl.rcParams['path.simplify'] = True
mpl.rcParams['path.simplify_threshold'] = 0.0
mplstyle.use('fast')

batch_file_path = "C:/Users/HADA/Desktop/all_sensor.bat"  # 배치 파일 경로를 지정하세요
process = subprocess.Popen(batch_file_path, shell=True)

process.wait()