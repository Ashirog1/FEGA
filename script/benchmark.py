import subprocess
import time
import os
import csv
import threading
import queue

root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

cpp_folder = os.path.join(root, "src")
cpp_filename = "bao.exe"
cpp_program = os.path.join(cpp_folder, cpp_filename)

from datetime import datetime
 
# storing the current time in the variab
curr = str(datetime.now().date())

input_folder = os.path.join(root, "txtData")  # replace with the path to the folder containing input files
num_iterations = 3  # number of times to run the C++ program for each input file
output_file = os.path.join(root, "result/results_run" + curr + ".csv")  # name of the output CSV file
print(output_file)
input_files = os.listdir(input_folder)

saved_result = dict()
#alphas = [0.1 * float(i) for i in range(0, 11)]
alphas = [0.3]


#files =['6.5.1.txt', '10.5.1.txt', '12.5.1.txt', '12.5.1.txt','20.10.1.txt', '50.20.1.txt', '100.30.1.txt', '150.30.1.txt', ]




q = queue.Queue()
for iter in range(0, num_iterations):
    for input_file in input_files:
        if ( '200' not in input_file):
            for alpha in alphas:
                q.put([iter, input_file, str(alpha)])

def worker():
    while True:
        all = q.get()
        if all is None:
            break
        iter, input_file, alpha = all
        #if (
        #    input_file[:3] == "100"
        #    or input_file[:3] == "200"
        #    or input_file[:3] == "150"
        #):
        #    continue
        # if (input_file != '6.5.1.txt'):
        #     continue
        if (input_file[0] != '6'):
            continue
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



threads = [ threading.Thread(target=worker) for _i in range(12) ]
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
            "Input File",
            "Iteration",
            "Learning Rate",
            "Execution Time(s)",
            "Program Result",
            "Drone Trip",
            "Avg Before",
            "Avg After",
            "Min Before",
            "Min After",
            "Max Before",
            "Max After",
            "Generation Result"
        ]
    )
    for key in saved_result:
        iter, input_file, alpha = key
        execution_time, result = saved_result[key]
        program_result = result.strip()
        program_result = program_result.replace('"', "")
        program_result = program_result.split(";")
        # program_result[2] = program_result[2].replace('"', '')

        writer.writerow(
            [
                input_file,
                iter + 1,
                int(float(alpha) * 10),
                "{:.6f}".format(execution_time),
                program_result[0],
                program_result[1],
                program_result[2],
                program_result[3],
                program_result[4],
                program_result[5],
                program_result[6],
                program_result[7],
                program_result[8]                
            ]
        )
