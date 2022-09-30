import sys
import pandas as pd
import os
import glob


def main():
  loc = os.getcwd()
  print("loc", loc)
  files = glob.glob(loc+'/txtData/*')
  for f in files:
    print(f)
    os.remove(f)

  print("reading path", loc + "/Data/" + "paramater.xlsx")
  try:
    paramater = pd.read_excel(loc + "/Data/" + "paramater.xlsx")
    print("paramater", paramater['dataset'])
    print("reading paramater.xlsx", paramater)
    print(paramater.dtypes)
  except Exception:
    print("error opening paramater.xlsx")
  valid_input = []
  for ind, row in paramater.iterrows():
    readfile = row["dataset"]
    for filename in os.listdir(loc+"/Data"):
      if filename.endswith(".xlsx"):
        continue
      if (str(readfile) == filename[:len(filename)-4]):
        df = pd.read_csv(loc+"/Data/"+filename)

        print("match", readfile)
        number_truck = row["number truck"]
        number_drone = row["number drone"]
        working_time = row["Woriking time"]  # :)))) wtf
        truck_capacity = row["Truck cappacity"]
        drone_capacity = row["Drone capacity"]
        drone_speed = row["Drone_speech"]
        truck_speed = row["Truck_speech"]
        drone_duration = row["Drone_duaration"]
        # try to get paramater from paramater.xlsx

        with open(loc+"/txtData/"+filename[:len(filename)-3] + "txt", "w") as outfile:
          try:
            print(int(number_truck), int(number_drone),
                  int(working_time), end="\n", file=outfile)
            print(int(truck_speed), int(drone_speed), int(truck_capacity), int(
                drone_capacity), int(drone_duration), end="\n", file=outfile)
            for ind, cus in df.iterrows():
              try:
                print(cus["x"], cus["y"], int(cus["low"]), int(
                    cus["upper"]), int(cus["weight"]), end="\n", file=outfile)
              except:
                for val in cus:
                  print(int(val), end=" ", file=outfile)
                print("", file=outfile)
            valid_input.append(filename[:len(filename)-4])
          except:
            print("error value type", filename)
        break


if __name__ == '__main__':
  main()
