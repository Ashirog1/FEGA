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

  files = glob.glob(loc+'/result/*')
  for f in files:
    print(f)
    os.remove(f)
  for f in os.listdir(loc+"/txtData"):
    print(f)
    if (f[:3] == "100" or f[:3] == "200"):
      continue
    if (f[:2] == "50"):
      continue
    if (f[:3] == "150"):
      continue
    with open(loc + "/txtData/" + f) as input:
      if (sum(1 for l in input) == 0):
        continue 
      run("cp " + loc + "/txtData/" + f + " " + loc + "/src/input.txt")
      os.chdir(os.path.expanduser("src"))
      try:
        run("sh run.sh")
      except:
        pass
      os.chdir(loc)

      #run("cat" + loc + "/src/input.txt")
      run("cp " + loc + "/src/result.log" + " " + loc + "/result/" + f[:len(f)-3] + "txt") 
      run("cp " + loc + "/src/result.csv" + " " + loc + "/result/" + f[:len(f)-3] + "csv") 
  return
  ## merge result folder into a single csv files
  df = pd.DataFrame()
  df.columns = ['input', 'best', 'worst', 'average', 'evaluate_call',
                'educate_call']
  '''
  for f in os.listdir(loc+"/result"):
    df = df.merge(pd.read_csv(f))
  '''
  df.to_csv(loc+"/result/"+"combined.csv")

if __name__ == '__main__':
  main()  