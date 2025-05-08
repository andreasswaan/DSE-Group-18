"""
Main project directory
"""

import pathlib

main_dir = pathlib.Path(__file__).parent.absolute()

if __name__ == '__main__':
    print(main_dir)