# ga-tsp

Run single test
```bash
cd src
g++ main.cpp && ./a.out 0 0 0 < input.txt
```
a.out take 3 paramater as the rate of genetic algorithm
(-1 is 0%, 0 is 50% and 1 is 100%).

Setup benchmark
```bash
cd script
python reformatData.py
python benchmark.py
```

```bash
g++ main.cpp -o a.exe && a.exe 5 5 2 < input.txt
```