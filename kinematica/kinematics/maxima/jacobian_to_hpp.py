#!/usr/bin/env python3

"""
Author: Emiel Bosman
File: Script to generate JacobianMatrix calculations using
a maxima script

Requirements:
Python 3
Maxima v 5.14 or higher

A maxima script that outputs a Jacobian with each matrix
index as a file with names matching [input_pattern]
Allowed variables in Jacobian formula are defined in
[template_code]
"""

from argparse import ArgumentParser
from string import Template
import glob
import os
import re
import subprocess

input_pattern=r"J[0-9]+"

template_start="""#ifndef KINEMATICS_JACOBIMATRIX_HPP
#define KINEMATICS_JACOBIMATRIX_HPP
#include <cmath>
namespace kinematics
{
"""

template_code=Template("""  inline double Calc$name(double Ct1,
        double St1,
        double Ct2,
        double St2,
        double Ct3,
        double St3,
        double Ct4,
        double St4,
        double Ct5,
        double St5,
        double Ct6,
        double St6,
        double Ct7,
        double St7)
        {
    // clang-format off
	return $formula
    // clang-format on
    }
""")

template_end="""
} // namespace kinematics
#endif // KINEMATICS_JACOBIMATRIX_HPP
"""

def main():
    output, mac_file, clean_output, verbose = get_arguments()
    clean(output)
    call_maxima(mac_file, verbose)
    print("Find files matching: {} in current directory".format(input_pattern))
    files = glob_re(input_pattern, os.listdir())
    export_to_cpp(files, output)
    if clean_output is True:
        clean(None)

def get_arguments():
    parser = ArgumentParser()
    parser.add_argument("-o", "--output", dest="output", default="JacobianMatrix.hpp",
                        help="Write cpp output to OUT", metavar="OUT")
    parser.add_argument("-m", "--maxima", dest="maxima", required=True,
                        help="maxima batch file to use", metavar="MAC")
    parser.add_argument("-c", "--clean", dest="clean", default=False,
                        help="Clean up generated formula files after generating cpp file",
                        action="store_true")
    parser.add_argument("-v", "--verbose", dest="verbose", default=False,
                        action="store_true", help="Do not suppress output for Maxima")

    args = vars(parser.parse_args())
    output = args["output"]
    maxima = args["maxima"]
    clean  = args["clean"]
    verbose = args["verbose"]
    return output, maxima, clean, verbose

def export_to_cpp(files, output):
    print("Exporting to CPP: {}".format(output))
    with open(output, 'w') as output_file:
        output_file.write(template_start)
        for filename in files:
            formula = pow_replace(filename)
            code = template_code.substitute(name = filename, formula = formula)
            print("Adding {} to CPP export".format(filename))
            output_file.write(code)
        output_file.write(template_end)
    print("Succesfully written to {}".format(output))

def pow_replace(filename, power = 2):
    # TODO Add logic to find all powers instead of only those matching the non-regex_pattern
    pow_search = "^{}".format(power)
    pow_pattern = "std::pow({term},{power})"

    with open(filename, "r") as file:
        text = file.read()
        index = text.find(pow_search)
        while (index is not -1):
            p_open, p_close = find_parentheses(text, index)

            text = (
                text[:p_open]
                + pow_pattern.format(
                        term = text[p_open:p_close + 1],
                        power = power
                    )
                + text[p_close + 1:index]
                + text[(index + len(pow_search)):]
            )
            index = text.find(pow_search)

    with open(filename, "w") as file:
        file.write(text)
    return text

def call_maxima(mac_file, verbose):
    maxima_call = ["maxima", "-b", mac_file]
    print("Call maxima: {}".format(str(maxima_call)))
    stdout = None
    if verbose is False:
        stdout = open(os.devnull,"w")
    subprocess.call(maxima_call, stdout=stdout)

def clean(output = None):
    print("Remove files matching {} and {}".format(input_pattern, output))

    if (output is not None and glob.glob(output)):
        os.unlink(output)
    files = glob_re(input_pattern, os.listdir())
    for file in files:
        os.unlink(file)

def glob_re(pattern, strings):
    found = list(filter(re.compile(pattern).match, strings))
    found.sort()
    return found

def find_parentheses(s, index):
    """ Find and return the location of the matching parentheses pairs in s.

    Given a string, s, return a dictionary of start: end pairs giving the
    indexes of the matching parentheses in s. Suitable exceptions are
    raised if s contains unbalanced parentheses.

    Based on code as found on https://scipython.com/blog/parenthesis-matching-in-python/

    """
    stack = []
    for i, c in enumerate(s):
        if c == '(':
            stack.append(i)
        elif c == ')':
            try:
                if (i == index-1):
                    return stack.pop(), i
                else:
                    stack.pop()
            except IndexError:
                raise IndexError('Too many close parentheses at index {}'
                                                                .format(i))
    if stack:
        raise IndexError('No matching close parenthesis to open parenthesis '
                         'at index {}'.format(stack.pop()))
    return None, None

if __name__ == "__main__":
    main()
