import logging
import datetime
from flask_socketio import SocketIO
import os

# Define the SocketIO instance
socketio = None  # This will be set later

# Custom log level for STATUS
STATUS_LEVEL_NUM = 25
logging.addLevelName(STATUS_LEVEL_NUM, "STATUS")

def status(self, message, *args, **kwargs):
    if self.isEnabledFor(STATUS_LEVEL_NUM):
        self._log(STATUS_LEVEL_NUM, message, args, **kwargs)

# Add the method to the logging.Logger class
logging.Logger.status = status

class WebSocketHandler(logging.Handler):
    def emit(self, record):
        # Only send `STATUS` level logs to the frontend
        if record.levelno == STATUS_LEVEL_NUM:
            msg = self.format(record)
            try:
                if socketio:
                    socketio.emit('log_message', {'message': msg})
                    print("log emitted", flush=True)
                else:
                    print("SocketIO instance not set, cannot emit logs", flush=True)
            except Exception as e:
                print(f"Failed to emit log over WebSocket: {e}")

class ExcludeRsyncFilter(logging.Filter):
    def filter(self, record):
        # Exclude logs that contain "rsync"
        return "rsync" not in record.getMessage()

# Logger configuration
def setup_logging(sio_instance):
    global socketio
    socketio = sio_instance

    log_dir = "/home/dishita/Desktop/UI_PX4/edhithaGCS-main-UI-server"
    os.makedirs(log_dir, exist_ok=True)

    # Get the current date and time for the log filename
    current_datetime = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output_file = f"../../LOGS/logs_{current_datetime}.txt"
    status_log_file = f"../../LOGS/stat/status_logs_{current_datetime}.txt"

    # Set up basic logging
    logging.basicConfig(level=logging.DEBUG,
                        format='%(asctime)s - %(levelname)s - %(message)s',
                        filename=output_file)

    # Define a StreamHandler to print log messages to the console
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    formatter = logging.Formatter('%(message)s')
    console_handler.setFormatter(formatter)

    # Add a file handler specifically for STATUS logs
    status_file_handler = logging.FileHandler(status_log_file)
    status_file_handler.setLevel(STATUS_LEVEL_NUM)
    status_file_handler.setFormatter(formatter)

    # Add the WebSocketHandler
    ws_handler = WebSocketHandler()
    ws_handler.setLevel(STATUS_LEVEL_NUM)  # Only STATUS level logs will be pushed
    ws_handler.setFormatter(formatter)

    # Set up the root logger
    root_logger = logging.getLogger()
    root_logger.addHandler(console_handler)
    root_logger.addHandler(status_file_handler)  # Add STATUS file handler
    root_logger.addHandler(ws_handler)  # Add the WebSocket handler

    console_handler.addFilter(ExcludeRsyncFilter())
    status_file_handler.addFilter(ExcludeRsyncFilter())
    ws_handler.addFilter(ExcludeRsyncFilter())

    # Set werkzeug logging level
    # werkzeug_log = logging.getLogger('werkzeug')
    # werkzeug_log.setLevel(logging.WARNING)

# Usage of custom `STATUS` log level
# Example: logger.status("This is a custom status log")
