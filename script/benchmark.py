import subprocess
import time
import os
import csv
import threading
import queue
import glob 

root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
print (root)

cpp_folder = os.path.join(root, "src")
cpp_filename = "bao.exe"
cpp_program = os.path.join(cpp_folder, cpp_filename)

#input_folder = os.path.join(root, "txtData_k-1_1")  # replace with the path to the folder containing input files

# input_folder = r"C:\Users\HoangHaPC\Desktop\huett\data_v3_saving_k"
# output_file = os.path.join(root, r"result\data_v3_saving_k.csv")  # name of the output CSV file

# input_folder = r"C:\Users\HoangHaPC\Desktop\huett\data_v3_saving_k-1_0"
# output_file = os.path.join(root, r"result\data_v3_saving_k-1_0.csv")  # name of the output CSV file

input_folder = r"C:\Users\HoangHaPC\Desktop\huett\data_v3_saving_k-1_1"
output_file = os.path.join(root, r"result\data_v3_saving_k-1_1.csv")


# input_folder = r"C:\Users\HoangHaPC\Desktop\huett\data_v3.3"
# output_file = os.path.join(root, r"result\data_v3.3.50 small.csv")  # name of the output CSV file


# input_folder = r"C:\Users\HoangHaPC\Desktop\huett\data_v3_saving_large/"
# output_file = os.path.join(root, r"result\data_v3_saving_large.csv") 

num_iterations = 5  # number of times to run the C++ program for each input file


input_files = os.listdir(input_folder)


saved_result = dict()
#alphas = [0.1 * float(i) for i in range(0, 11)]
alpha = 0.1
q = queue.Queue()
for iter in range(0, num_iterations):
    for input_file in input_files:
    #    if "10.3k" in input_file:
         
#       for alpha in alphas:
        q.put([iter, input_file, str(alpha)])

def worker():
    while True:
        all = q.get()
        if all is None:
            break
        iter, input_file, alpha = all
        # if (
        #     input_file[:1] == "6"
        #     or input_file[:2] == "10"
        #     or input_file[:2] == "12"
        #     or input_file[:2] == "20"
        #     or input_file[:2] == "50"
        # ):
        #     continue
        input_path = os.path.join(input_folder, input_file)
        start_time = time.time()
        print(
            input_file,
            iter,
            "alpha=",
            alpha,
        )
        with open(input_path) as infile:
            result = subprocess.run(
                [
                    cpp_program,
                    alpha,
                ],
                stdin=infile,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            end_time = time.time()

            # print(result.stdout)
            execution_time = end_time - start_time
            saved_result[(iter, input_file, alpha)] = (
                execution_time,
                result.stdout,
            )




threads = [ threading.Thread(target=worker) for _i in range(10) ]
for thread in threads:
    thread.start()
    q.put(None)  # one Sentinel marker for each thread
for thread in threads:
    thread.join()

# log to csv file

with open(output_file, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(
        [
            "Iteration",
            "Input File", 
            "Learning Rate",
            "Execution Time(s)",
            "Program Result",
            "Drone Trip",
            "Generation Result",
        ]
    )
    for key in saved_result:
        try:
            iter, input_file, alpha = key
            execution_time, result = saved_result[key]
            program_result = result.strip()
            program_result = program_result.replace('"', "")
            program_result = program_result.split(";")
            # program_result[2] = program_result[2].replace('"', '')
            #print (key)
            writer.writerow(
                [
                    iter + 1,
                    input_file,
                    int(float(alpha) * 10),
                    "{:.6f}".format(execution_time),
                    program_result[0],
                    program_result[1],
                    program_result[2],
                ]
            )
        except:
            pass
     
    print(output_file)

import pandas as pd
df  = pd.read_csv(output_file)
for index, row in df.iterrows():
    a = (row['Input File'])  
    customer_number = int(a.split(".")[0])
    df.at[index, 'Customer_number'] = customer_number   
    a = (row['Generation Result'])
    list_from_string = eval(a)
    df.at[index, 'Program Result_2'] = max(list_from_string) 

df.to_csv(output_file[:-4]+str(time.time())+".csv", index=False)
