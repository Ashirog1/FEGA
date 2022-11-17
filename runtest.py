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
  run("pwd")
  for f in os.listdir(loc+"/txtData2"):
    print(f)
    with open(loc + "/txtData2/" + f) as input:
      if (sum(1 for l in input) == 0):
        continue 
      run("cp " + loc + "/txtData2/" + f + " " + loc + "/src/input.txt")
      os.chdir(os.path.expanduser("src"))
      try:
        run("sh run.sh")
      except:
        pass
      os.chdir(loc)
      #run("cat" + loc + "/src/input.txt")
      run("cp " + loc + "/src/result.log" + " " + loc + "/result/" + f[:len(f)-3] + "txt") 
      

if __name__ == '__main__':
  main()  