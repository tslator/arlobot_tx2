from __future__ import print_function

import package_main

package_main.banner(__package__)
tests = package_main.process(__file__)
if tests:
    for test in tests:
        print(test)
        #exec(test)
else:
    print("No tests found!")

# --- EOF ---
