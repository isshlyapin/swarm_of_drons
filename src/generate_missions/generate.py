import csv
import random

PROBABILITY_FILE = "probability4.csv"
MISSIONS_FILE    = "missions.csv"

AMOUNT_DRONPORTS = 4
AMOUNT_MISSIONS  = 120

with open(PROBABILITY_FILE) as f:
    reader = csv.reader(f)
    missions = []
    
    for i in range(1, AMOUNT_DRONPORTS + 1):
        row = next(reader)
        for j in range(1, AMOUNT_DRONPORTS + 1):
            nums = int(AMOUNT_MISSIONS * float(row[j-1]))
            for k in range(0, nums):
                missions.append([f"d{i}", f"d{j}"])

with open(MISSIONS_FILE, 'w') as f:
    data = [{"index1": i[0], "index2": i[1], "time_appearance": 0} for i in missions]
    random.shuffle(data)
    writer = csv.DictWriter(f, fieldnames=list(data[0].keys()), quoting=csv.QUOTE_NONNUMERIC)
    writer.writeheader()
    for d in data:
        writer.writerow(d)
