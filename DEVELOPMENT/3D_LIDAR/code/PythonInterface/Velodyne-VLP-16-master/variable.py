# REAL TIME SETTING --------------------
time_curr = 0
time_cnt = 0
time_stime = 0
time_ts = 0.15
time_final = 5

# SETTING
framecut = 0.1

# SHAREDMEMORY
shared_memory_name = "CAMERA"

# --------------------------------------

LASER_ANGLES = [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]
NUM_LASERS = 16
EXPECTED_PACKET_TIME = 0.001327  # valid only in "the strongest return mode"
EXPECTED_SCAN_DURATION = 0.1
DISTANCE_RESOLUTION = 0.002
ROTATION_RESOLUTION = 0.01
ROTATION_MAX_UNITS = 36000
VLP_START_BYTE = 2
VLP_AZIMUTH_BYTE = 2
VLP_DIST_BYTE = 2
VLP_REFLEC_BYTE = 1
VLP_BLOCK_BYTE = 100
VLP_BLOCK_BYTE = 100

HOST = "192.168.1501"
PORT = 2368

formatter = '[%(asctime)s][%(filename)s:%(lineno)s][%(levelname)s][%(message)s]'

LOGGING_CONFIG = {
    'version': 1,
    'disable_existing_loggers': False,  # this fixes the problem
    'formatters': {
        'standard': {
            'format': formatter,
        },
    },
    'handlers': {
        'default': {
            'level': 'DEBUG',
            'class': 'logging.StreamHandler',
            'formatter': 'standard'
        },
        "debug_file_handler": {
            "class": "logging.handlers.TimedRotatingFileHandler",
            "level": "DEBUG",
            "formatter": "standard",
            "filename": "./logs/lidar.log",
            "when": "D",
            "interval": 1,
            "backupCount": 30,
            "encoding": "utf8"
        },
    },
    'loggers': {
        '': {
            'handlers': ["default", 'debug_file_handler'],
            'level': 'DEBUG',
            'propagate': False
        },
    }
}

# logging.config.dictConfig(LOGGING_CONFIG)
# logger = logging.getLogger("")


def printSetting() :
    
    print("\n\n------------------------------------------------------------------")
    print("                      PROGRAM START...!                             \n")
    print("             - SAMPLING TIME  :  {} [sec]".format(time_ts))                       
    print("             - FINAL TIME     :  {} [sec]".format(time_final))
    print("------------------------------------------------------------------\n")