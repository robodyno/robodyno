#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""utils.py
Time    :   2022/10/19
Author  :   song 
Version :   1.0
Contact :   zhaosongy@126.com
License :   (C)Copyright 2022, robottime / robodyno

Command line tools utilities
"""

from colorama import Fore

colorama_color_dict = {
    'black': Fore.BLACK,
    'red': Fore.RED,
    'green': Fore.GREEN,
    'yellow': Fore.YELLOW,
    'blue': Fore.BLUE,
    'magenta': Fore.MAGENTA,
    'cyan': Fore.CYAN,
    'white': Fore.WHITE,
}

def colored(str, color):
    """create colored command line string
    
    Args:
        str: origin string
        color: color string
    
    Returns:
        colored string
    """
    return colorama_color_dict[color] + str + Fore.RESET