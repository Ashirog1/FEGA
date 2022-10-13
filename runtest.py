import sys
import pandas as pd
import os
import glob
import subprocess

def main():
  loc = os.getcwd()
  print("loc", loc)
  files = glob.glob(loc+'/txtData/*')
  for f in files:
    subprocess.r 