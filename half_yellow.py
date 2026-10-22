# SECOND HALF — shoot at the YELLOW goal, defend BLUE.
# Run THIS file for the half where you must score in yellow.
# (The two halves use opposite goal colors, so two wrappers.)
import sys
from robocup import main

sys.argv = [sys.argv[0], "--half", "2"] + sys.argv[1:]
main()
