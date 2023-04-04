import subprocess
import time
import os
import csv
import threading
import queue

cpp_folder = "src"
cpp_filename = "a.out"
cpp_program = os.path.join(cpp_folder, cpp_filename)

input_folder = "txtData/"  # replace with the path to the folder containing input files
num_iterations = 10  # number of times to run the C++ program for each input file
output_file = "results.csv"  # name of the output CSV file

input_files = os.listdir(input_folder)

saved_result = dict()

q = queue.Queue()
for iter in range(0, num_iterations):
    for input_file in input_files:
        for alpha in range(0, 150, 50):
            for beta in range(0, 150, 50):
                for omega in range(0, 150, 50):
                    q.put([iter, input_file, alpha, beta, omega])

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
            "init rate=",
            alpha / 50 - 1,
            "cross over=",
            beta / 50 - 1,
            "mutation=",
            omega / 50 - 1,
        )
        with open(input_path) as infile:
            result = subprocess.run(
                [
                    cpp_program,
                    str(alpha / 50 - 1),
                    str(beta / 50 - 1),
                    str(omega / 50 - 1),
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
            "Init rate",
            "Crossover rate",
            "Mutation rate",
            "Execution Time(s)",
            "Program Result",
            "Drone Trip",
            "Generation Result",
        ]
    )
    for key in saved_result:
        iter, input_file, alpha, beta, omega = key
        execution_time, result = saved_result[key]
        program_result = result.strip()
        program_result = program_result.replace('"', "")
        program_result = program_result.split(";")
        # program_result[2] = program_result[2].replace('"', '')

        writer.writerow(
            [
                input_file,
                iter + 1,
                alpha,
                beta,
                omega,
                "{:.6f}".format(execution_time),
                program_result[0],
                program_result[1],
                program_result[2],
            ]
        )
