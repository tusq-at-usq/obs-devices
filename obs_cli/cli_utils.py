"""
A collection of command line interface helper functions
"""

import sys, shutil, re
import numpy as np

# Get the screen width
WIDTH = shutil.get_terminal_size((80, 20)).columns - 10


def progress_print(message, end=""):
    "Produce some aesthetically pleasing print statements"
    sys.stdout.write("\r")
    l = len(message)
    message_out = "".join([message] + ["."] * (WIDTH - l + 2) + [end])
    sys.stdout.write(message_out)


def print_centred(message):
    "Print message centred on the screen"
    white_space = "".join([" "] * (int(WIDTH // 2) - int(len(message) // 2)))
    message_out = "".join([white_space, message, white_space])
    print(f"\n{message_out}\n")


def clean_input(dirty_list, extra_removes=set(), do_not_removes=set()):
    "Removes useless entries from a list (as a result of parsing)"
    default_removes = {" ", "\n", "\t", None, ""}
    removes = default_removes.union(extra_removes) - do_not_removes

    squeaky_clean_list = [val for val in dirty_list if not val in removes]
    return squeaky_clean_list


def multiple_replace(text, blemishes={" ", "\t", "\n", '"', "'"}):
    "Shamelessly stolen from https://stackoverflow.com/questions/6116978"
    rep = dict((re.escape(k), "") for k in blemishes)
    pattern = re.compile("|".join(rep.keys()))
    return pattern.sub(lambda m: rep[re.escape(m.group(0))], text)


def find_nearest(array, value):
    "Determine the nearest value and index of an array to <value>"
    idx = (np.abs(array - value)).argmin()
    return array[idx], idx


def process_arguments(entries, default_args={}):
    "processes a raw string and splits the fields into arguments"
    ""
    args = {}
    err_string = ""

    for arg in entries:
        if not arg.startswith("-"):
            err_string += f"syntax error: {arg} is invalid input"
        elif arg.startswith("--"):
            err_string = f"argument {arg} not understood. Arguments use a single hyphon"

        # work out if its an argument or a flag
        split = arg.split("=")
        if len(split) == 1:
            args[arg[1:]] = True
        else:
            args[split[0][1:]] = split[1]

    return args, err_string


def split_fields_arguments(fields, default_args):
    "Seperates a list of inputs into field strings and arguments"

    # seperate flags from positional arguments
    for i, field in enumerate(fields):
        if field.startswith("-"):
            args, err_string = process_arguments(fields[i:], default_args)
            fields = fields[0:i]
            return fields, args, err_string

    return fields, {}, ""
