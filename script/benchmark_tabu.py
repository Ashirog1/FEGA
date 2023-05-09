import subprocess
import time
import os
import csv
import threading
import queue

cpp_folder = "../src"
cpp_filename = "a.out"
cpp_program = os.path.join(cpp_folder, cpp_filename)

input_folder = "../Data_Shetty_v2/"  # replace with the path to the folder containing input files
num_iterations = 5  # number of times to run the C++ program for each input file
output_file = "../result/results.csv"  # name of the output CSV file

input_files = os.listdir(input_folder)

saved_result = dict()

q = queue.Queue()
for iter in range(0, num_iterations):
    for input_file in input_files:
        q.put([iter, input_file, 5, 5, 2])

def worker():
    while True:
        all = q.get()
        if all is None:
            break
        iter, input_file, alpha, beta, omega = all
        if (
            input_file[:3] == "100"
            or input_file[:3] == "200"
            or input_file[:3] == "150"
        ):
            continue
        input_path = os.path.join(input_folder, input_file)
        start_time = time.time()
        print(
            input_file,
            iter,
        )
        with open(input_path) as infile:
            result = subprocess.run(
                [
                    cpp_program,
                    str(alpha),
                    str(beta),
                    str(omega)
                ],
                stdin=infile,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            end_time = time.time()

            # print(result.stdout)
            execution_time = end_time - start_time
            saved_result[(iter, input_file, alpha, beta, omega)] = (
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
            "Input File",
            "Iteration",
            "Time(s)",
            "Valid Solution",
            "Program Result",
            "Check Valid call", 
        ]
    )
    for key in saved_result:
        iter, input_file, alpha, beta, omega = key
        execution_time, result = saved_result[key]
        program_result = result.strip()
        program_result = program_result.replace('"', "")
        program_result = program_result.split(",")
        print(program_result)
        writer.writerow(
            [
                input_file,
                iter + 1,
                "{:.6f}".format(execution_time),
                program_result[0],
                program_result[1],
                program_result[2],
            ]
        )
