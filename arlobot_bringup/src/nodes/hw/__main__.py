# The purpose of this main file is to run modules tests
# Each module will expose a module test function which
# can be invoked via the package main, e.g.:
#
#    python -m nodes.devices <module> <test arguments>
#
# argument parsing will be needed

import sys
sys.path.append('scripts')
try:
    import package_main
except ImportError as err:
    print("Module Tests must be run from parent directory of nodes and scripts.", err)

package_main.banner(__package__)
mod_tests = package_main.process(__file__)
if mod_tests:
    for mod, test in mod_tests:
        print(test)
        print("--------{}------------".format('-' * len(mod)))
        print("Running {} Module Test".format(mod))
        exec(test)
        print("{} Module Test Complete".format(mod))
        print("--------{}-------------".format('-' * len(mod)))

else:
    print("No tests found!")
