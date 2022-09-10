import sys
import pandas as pd
import os

def main():
  loc = os.getcwd()
  print("loc", loc)

  print("reading path", loc + "/Data/" + "paramater.xlsx")
  try:
    paramater = pd.read_excel(loc + "/Data/" + "paramater.xlsx")
    print("reading paramater.xlsx", paramater)
  except Exception:
    print("error opening paramater.xlsx")

  for filename in os.listdir(loc+"/Data"):
    if filename.endswith(".xlsx"):
      continue

    print("reading", filename)

    df = pd.read_csv(loc+"/Data/"+filename)
    # try to get paramater from paramater.xlsx
    try:
      param = paramater.loc[paramater.dataset == filename[:len(filename)-3]]
      print("param", param)
    except Exception:
      print("error opening paramater.xlsx")

    with open(loc+"/txtData/"+filename[:len(filename)-3] + "txt", "w") as outfile:
      print("something", file=outfile)


if __name__ == '__main__':
  main()