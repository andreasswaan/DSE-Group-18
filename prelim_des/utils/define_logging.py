import logging

logging.basicConfig(
    format="{asctime} - {levelname} - {message}",
    style="{",
    datefmt="%Y-%m-%d %H:%M",
    filename="app.log",
    encoding="utf-8",
    filemode="a",
    level=logging.INFO,  # Set the logging level to DEBUG to capture all messages
)


if __name__ == "__main__":
    # Test logging setup
    logging.info("Logging setup complete. This is an info message.")
    logging.warning("This is a warning message.")
    logging.error("This is an error message.")
    logging.debug("This is a debug message.")
    logging.critical("This is a critical message.")
