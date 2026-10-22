# FIRST HALF — shoot at the BLUE goal, defend YELLOW.
# The referee tells you which side is which at kickoff.  Run THIS
# file for the half where you must score in blue.
import sys
from robocup import main

sys.argv = [sys.argv[0], "--half", "1"] + sys.argv[1:]
main()
