import sys
import pandas as pd
import os
import glob
import subprocess

def main():
  def run(str):
    print(str)
    os.system(str)
  loc = os.getcwd()
  print("loc", loc)
  run(loc + "/src/./main" + " < " + loc + "/src/input.txt")
  return
  for f in os.listdir(loc+"/txtData"):
    print(f)
    with open(loc + "/txtData/" + f) as input:
      if (sum(1 for l in input) == 0):
        continue 
      os.system("cp " + loc + "/txtData/" + f + " " + loc + "/src/input.txt")
      os.system("sh " + "run.sh")
      os.system("cp " + loc + "/src/debug.log" + " " + loc + "/result/" + f[:len(f)-3] + "txt") 

if __name__ == '__main__':
  main()  