import os
import glob
from prompt_toolkit.completion import WordCompleter


def initialize_autocompletion():

    autocompletion = WordCompleter([], WORD=True)

    autocompletion.words += [
        "set",
        "start",
        "stop",
        "set",
        "SP",
        # "load",
        "limit"
        "home",
        "az",
        "el",
        "reset",
        "setpoint",
        "manual",
        "view"
        "stream",
        "save",
        "llim",
        "ulim",
        "limits",
        "clahe",
        "on",
        "off",
        "find",
        # "starcal",
        # "orientation",
        # "fov",
    ]

    return autocompletion


def add_files_to_autocompletion(autocompletion):
    filenames = glob.glob("./*.txt")
    autocompletion.words += filenames
    return autocompletion
