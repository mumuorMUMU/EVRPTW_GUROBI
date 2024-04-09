from gurobipy import *
import numpy as np

model = Model("EVRPTW")

num_s, num_c, num_depot = 0, 0, 0
xc, yc, q, e, l, s = [], [], [], [], [], []

# read data
with open("E-VRPTW Instances/c206C5.txt") as f:
    meet_empty_line = False
    for line in f.readlines()[1:]:
        print(f"line[0]:{line[0]}")
        if line == "\n":
            meet_empty_line = True
            continue
        if not meet_empty_line:
            if line[0] in "DCS":
                # instance information
                name, type_, x, y, demand, ready_time, due_time, service_time = line.split()
                # set dummy vertices of set of stations
                repeat_count = 2 if type_ == 'f' else 1
                for _ in range(repeat_count):
                    xc.append(float(x))
                    yc.append(float(y))
                    q.append(float(demand))
                    e.append(float(ready_time))
                    l.append(float(due_time))
                    s.append(float(service_time))
                if type_ == "f":
                    num_s += 1
                elif type_ == "c":
                    num_c += 1
                else:
                    num_depot += 1
            else:
                raise Exception("wrong file")
        else:
            # vehicle information
            end_num = float(line[line.find("/")+1:-2])
            if line[0] == "Q": # Battery capacity
                Q = end_num
            elif line[0] == "C": # Cargo capacity
                C = end_num
            elif line[0] == "r": # Charge consumption rate
                h = end_num
            elif line[0] == "g": # Charge rate
                g = end_num
            elif line[0] == "v": # Speed
                V = end_num
            else:
                raise Exception("wrong file")
    # set dummy vertices of depot
    xc.append(xc[0])
    yc.append(yc[0])
    q.append(q[0])
    e.append(e[0])
    l.append(l[0])
    s.append(s[0])

"""set"""
depot_0 = [0]
depot_n = [2*num_s + num_c + 1]
#print(depot_n)

stations = [i for i in range(1, 2*num_s+1)]
#print(stations)
stations_0 = depot_0 + stations

customers = [i for i in range(2*num_s+1, 2*num_s+num_c+1)]
#print(customers)
customers_0 = depot_0 + customers

stations_customers = stations + customers
stations_customers_0 = depot_0 + stations_customers
stations_customers_n = stations_customers + depot_n
total = depot_0 + stations + customers + depot_n
# print(total)

A = [(i, j) for i in stations_customers_0 for j in stations_customers_n]
Dist = {(i, j): np.hypot(xc[i]-xc[j], yc[i]-yc[j]) for i, j in A} # distance matrix
Time = {(i, j): (np.hypot(xc[i]-xc[j], yc[i]-yc[j]))/V for i, j in A} # time matrix

"""decision var"""
t = model.addVars(total, vtype = GRB.CONTINUOUS, name = "arrive time")
u = model.addVars(total, vtype = GRB.CONTINUOUS, name = "remaining cargo")
b = model.addVars(total, vtype = GRB.CONTINUOUS, name = "remaining battery")
x = model.addVars(A, vtype = GRB.BINARY, name = "arcs")

"""constraints"""
# each customer is visited exactly once
model.addConstrs((quicksum(x[i, j] for j in stations_customers_n if i!=j) == 1 for i in customers), name = "c0")

# the number of vehicles, set this constraint for test
# model.addConstr((quicksum(x[0, j] for j in stations_customers_n) <= 1), name = "c11")

# handle the connectivity of visits to stations
model.addConstrs((quicksum(x[i, j] for j in stations_customers_n if i!=j) <= 1 for i in stations), name = "c1")

# after a vehicle arrives at a customer or station it has to leave for another destination
model.addConstrs((quicksum(x[j,i] for i in stations_customers_n if i!=j) - quicksum(x[i,j] for i in stations_customers_0 if i!=j) == 0  for j in stations_customers), "c2")

# time feasibility for arcs leaving customers and depots
model.addConstrs((t[i] + (Time[i, j] + s[i])* x[i, j] - l[0]*(1- x[i, j]) <= t[j] for i in customers_0 for j in stations_customers_n if i!=j), name = "c3")

# time feasibility for arcs leaving stations
model.addConstrs((t[i] + Time[i, j]*x[i, j] + g*(Q-b[i]) - (l[0]+Q*g)*(1-x[i, j]) <= t[j] for i in stations for j in stations_customers_n if i!=j), name = "c4")

# every node is visited within the time window
model.addConstrs((t[j] <= l[j] for j in total), name = "c5")
model.addConstrs((e[j] <= t[j] for j in total),name = "c6")

# demand fulfillment at all customers
model.addConstrs((u[j] <= u[i] -q[i]*x[i,j] + C*(1-x[i, j]) for i in stations_customers_0 for j in stations_customers_n if i!=j), name = "c7")
model.addConstrs((u[i] <= C for i in depot_0), name = "c8")

# battery feasibility for arcs leaving customers
model.addConstrs((b[j] <= b[i] - (h*Dist[i, j])*x[i,j] + Q*(1-x[i, j]) for i in customers for j in stations_customers_n if i!=j), name = "c9")
# battery feasibility for arcs leaving stations and depots
model.addConstrs((b[j] <= Q - (h*Dist[i, j])*x[i,j]  for i in stations_0 for j in stations_customers_n if i!=j), name = "c10")

"""objective"""
dist = quicksum(Dist[i, j]*x[i, j] for i in stations_customers_0 for j in stations_customers_n if i!=j)
penalty = 500*quicksum(x[0, j] for j in stations_customers_n)
obj = dist + penalty
model.setObjective(obj, GRB.MINIMIZE)

"""params"""
model.Params.TimeLimit = 500

"""optimize"""
model.update()
model.optimize()

"""solution"""
print(f"Status:{model.Status}")
print(f"Objective:{model.ObjVal}")
print(f"CPU time:{model.Runtime}")

print(f'The total distance : {dist.getValue()}')
print(f'The penalty for the number of vehicle: {penalty.getValue()}')

"""routes"""
optimal_x = model.getAttr('X', x)

for i in stations_customers_0:
    for j in stations_customers_n:
        if optimal_x[i, j] == 1:
            print(f"travel from {i} to {j}")














