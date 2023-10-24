from collections import defaultdict, namedtuple
from pyomo.util.infeasible import log_infeasible_constraints
import pyomo.environ as pyomo
import pandas
import logging
from timeit import default_timer as timer
import argparse
import os
import math

parser = argparse.ArgumentParser()
parser.add_argument("nGat")
parser.add_argument("nPla")
parser.add_argument("nDev")
parser.add_argument("seed")
parser.add_argument("qos_bound")

args = parser.parse_args()

os.system("date")

DEBUG = False
VERBOSE = True
PACKET_SIZE = 400  # bits
QOS_LOWER_BOUND = float(args.qos_bound)
LOSS_HIGHER_BOUND = 4
TRANSMISSION_PROBABILITY = 0.01
MAX_RKC = 6835.94
MIN_RKC = 183.11
MAX_DELAY = PACKET_SIZE / MIN_RKC

startPre = timer()


# NS-3 "thesis-experimentation" pre-processed files 
cwd = os.getcwd()
path_output = cwd + "/data/model/output/"
path_data = cwd + "/data/model/"
gateway_data = cwd + "/data/placement/"
devices_data = cwd + "/data/placement/"

prefix = "opt_gap"

end_device_position_file = devices_data + "endDevices_LNM_Placement_" + str(args.seed) + "s+" + str(args.nDev) + "d.dat"
#"endDevices_LNM_Placement_1s+10d.dat"
if args.nPla == '1':
    gateway_position_file = gateway_data + "equidistantPlacement_" + str(args.nGat) + ".dat"
else:
    gateway_position_file = gateway_data + "equidistantPlacement_" + str(args.nGat) + "x" + str(args.nPla) + ".dat"

slice_association_file = path_data + "skl_" + \
                         str(args.seed) + "s_" + \
                         str(args.nGat) + "x" + \
                         str(args.nPla) +"Gv_" + \
                         str(args.nDev) + "D.dat"

# plr1_file = path_data + "plrI_" + \
#                          str(args.seed) + "s_" + \
#                          str(args.nGat) + "x" + \
#                          str(args.nPla) +"Gv_" + \
#                          str(args.nDev) + "D.dat"

#DEBUG filenames
if DEBUG:
    print(end_device_position_file)
    print(gateway_position_file)
    print(slice_association_file)

#exit()

Position = namedtuple('Position', ['x', 'y', 'z'])
Configuration = namedtuple('Configuration', ['sf', 'tp'])
Gateway = namedtuple('Gateway', ['bandwidths', 'max_datarates'])

# -------------------------------
# Processing End Device Positions
# -------------------------------

end_device_position_df = pandas.read_csv(end_device_position_file, names=['x', 'y', 'z'],
                                         sep=" ", index_col=False)

end_device_positions = {}
for row in end_device_position_df.itertuples():
    end_device_positions[row.Index] = Position(x=row.x, y=row.y, z=row.z)

if VERBOSE:
    print('End Device Positions: ')
    print(end_device_positions)
    print('------------------------------------------------------------------------------')

# ---------------------------------------
# Processing End Device Slice Association
# ---------------------------------------

slice_association_df = pandas.read_csv(slice_association_file, names=['device', 'slice'],
                                       sep=" ", index_col=False)

slice_associations = {}
for row in slice_association_df.itertuples():
    slice_associations[row.device] = row.slice

if VERBOSE:
    print('Slice Associations: ')
    print(slice_associations)
    print('------------------------------------------------------------------------------')

# -------------------------------
#       Processing Slices
# -------------------------------

slices = {0: 'slice1', 1: 'slice2', 2: 'slice3'}

if VERBOSE:
    print('Slices: ')
    print(slices)
    print('------------------------------------------------------------------------------')

# -----------------------------
# Processing Gateway Positions
# -----------------------------

gateway_position_df = pandas.read_csv(gateway_position_file, names=['x', 'y', 'z'],
                                      sep=" ", index_col=False)

gateway_positions = {}
for row in gateway_position_df.itertuples():
    gateway_positions[row.Index + len(end_device_positions)] = Position(x=row.x, y=row.y, z=row.z)

if VERBOSE:
    print('Gateway Positions: ')
    print(gateway_positions)
    print('------------------------------------------------------------------------------')

# -----------------------------
#      Processing Gateways
# -----------------------------

gateways = {0: Gateway(bandwidths=[125000.0 for i in enumerate(slices)],
                       max_datarates=[15197.75390625 for i in enumerate(slices)])}

if VERBOSE:
    print('Gateways: ')
    print(gateways)
    print('------------------------------------------------------------------------------')

# -----------------------------
#   Processing Configurations
# -----------------------------

spreading_factors = range(7, 13)
transmission_powers = range(2, 16, 2)
configurations = {}
sensitivity = {7: -130.0, 8: -132.5, 9: -135.0, 10: -137.5, 11: -140.0, 12: -142.5} 
# sensitivity = {7: -124.0, 8: -127.0, 9: -130.0, 10: -133.0, 11: -135.0, 12: -137.0} # Simulator params
referencePrx = 10.0
referenceDistance = 1.0
attenuationExponent = 3.76
for idx, value in enumerate(Configuration(sf, tp)
                            for sf in spreading_factors
                            for tp in transmission_powers):
    configurations[idx] = value

# Symbol Rate SF_c [bits/s]
symbol_rates = {}
for idx, configuration in configurations.items():
    if configuration.sf == 7:
        # symbol_rates[idx] = 3417.97 #CR 4
        # symbol_rates[idx] = 4557.29 #CR 3
        # symbol_rates[idx] = 3906.25 #CR 2
        symbol_rates[idx] = 5468.75  #CR 1
    elif configuration.sf == 8:
        # symbol_rates[idx] = 1953.13 #CR 4
        # symbol_rates[idx] = 2232.14 #CR 3
        # symbol_rates[idx] = 2604.17 #CR 2
        symbol_rates[idx] = 3125.00 #CR 1
    elif configuration.sf == 9:
        # symbol_rates[idx] = 1098.63 #CR 4
        # symbol_rates[idx] = 1255.58 #CR 3
        # symbol_rates[idx] = 1464.84 #CR 2
        symbol_rates[idx] = 2197.27 #CR 1
    elif configuration.sf == 10:
        # symbol_rates[idx] = 610.35 #CR 4
        # symbol_rates[idx] = 697.54 #CR 3
        # symbol_rates[idx] = 813.80 #CR 2
        symbol_rates[idx] = 976.56 #CR 1
    elif configuration.sf == 11:
        # symbol_rates[idx] = 335.69  # CR 4
        # symbol_rates[idx] = 383.65  # CR 3
        # symbol_rates[idx] = 447.59  # CR 2
        symbol_rates[idx] = 537.11  # CR 1
    elif configuration.sf == 12:
        # symbol_rates[idx] = 183.11  # CR 4
        # symbol_rates[idx] = 209.26  # CR 3
        # symbol_rates[idx] = 244.14  # CR 2
        symbol_rates[idx] = 292.97  # CR 1

if VERBOSE:
    print('Configurations: ')
    print(configurations)
    print('\nSymbol Rates: ')
    print(symbol_rates)
    print('------------------------------------------------------------------------------')

# -----------------------------
#      Processing PLR'
# -----------------------------

# plr1_df = pandas.read_csv(plr1_file, names=['device', 'gateway', 'sf', 'tp'],
#                           sep=" ", index_col=False)

# inverse_configurations = {value: key for key, value in configurations.items()}
# plr1 = defaultdict(list)
# for row in plr1_df.itertuples():
#     config_id = inverse_configurations[Configuration(row.sf, row.tp)]
#     plr1[(row.device, row.gateway)].append(config_id)

# if VERBOSE:
#     print('PLR\': ')
#     print(plr1)
#     print('------------------------------------------------------------------------------')

# -----------------------------
# ------ Building Model -------
# -----------------------------

model = pyomo.ConcreteModel()

model.M = pyomo.Set(initialize=gateways.keys())
model.P = pyomo.Set(initialize=gateway_positions.keys())
model.K = pyomo.Set(initialize=end_device_positions.keys())
model.C = pyomo.Set(initialize=configurations.keys())
model.SF = pyomo.Set(initialize=spreading_factors)

model.x = pyomo.Var(model.M, model.P, model.K, model.C, domain=pyomo.Binary)
model.ceil1 = pyomo.Var(model.M, model.P, domain=pyomo.Binary)
model.ceil2 = pyomo.Var(model.M, model.P, domain=pyomo.Binary)
model.ceil3 = pyomo.Var(model.M, model.P, domain=pyomo.Binary)
model.ceil4 = pyomo.Var(model.M, model.SF, domain=pyomo.Binary)

# ------------------
# Objective Function
# ------------------

model.ceil1_lower_bound = pyomo.ConstraintList()
model.ceil1_higher_bound = pyomo.ConstraintList()

expression = 0
for gateway in model.M:
    for position in model.P:
        ceil_expr = sum(
            model.x[gateway, position, device, config] / len(model.K)
            for device in model.K
            for config in model.C
        )
        model.ceil1_lower_bound.add(model.ceil1[gateway, position] - ceil_expr >= 0.0)
        model.ceil1_higher_bound.add(model.ceil1[gateway, position] - ceil_expr <= 1.0 - (1 / len(model.K)))
        expression += model.ceil1[gateway, position]

model.OBJECTIVE = pyomo.Objective(expr=expression, sense=pyomo.minimize)

# ------------------------
# One Gateway Per Position
# ------------------------

# model.ceil3_lower_bound = pyomo.ConstraintList()
# model.ceil3_higher_bound = pyomo.ConstraintList()
# model.single_gateway_per_position = pyomo.ConstraintList()
# for position in model.P:
#     expression = 0
#     for gateway in model.M:
#         ceil_expr = sum(
#             model.x[gateway, position, device, config] / len(model.K)
#             for device in model.K
#             for config in model.C
#         )
#         model.ceil3_lower_bound.add(model.ceil3[gateway, position] - ceil_expr >= 0.0)
#         model.ceil3_higher_bound.add(model.ceil3[gateway, position] - ceil_expr <= 1.0 - (1 / len(model.K)))
#         expression += model.ceil3[gateway, position]
#     model.single_gateway_per_position.add(expression <= 1.0)

# -----------------------------------------------------
# Single Gateway, Position and Configuration per Device
# -----------------------------------------------------

model.single_match_for_device = pyomo.ConstraintList()
for device in model.K:
    expression = sum(
        model.x[gateway, position, device, config]
        for gateway in model.M
        for position in model.P
        for config in model.C
    )
    model.single_match_for_device.add(expression == 1)

# --------------------------
# Gateway Capacity per Slice
# --------------------------

model.gateway_capacity = pyomo.ConstraintList()
for position in model.P:
    for slice in slices.keys():
        expression = sum(
            model.x[gateway, position, device, config] *
            (configurations[config].sf * gateways[gateway].bandwidths[slice] /
             (2 ** configurations[config].sf))
            for device in model.K
            for gateway in model.M
            for config in model.C
            # Equivalent to S(k,l)
            if slice_associations[device] == slice
        )
        for gateway in model.M:
            model.gateway_capacity.add(expression <= gateways[gateway].max_datarates[slice])

# model.gateway_capacity.add(expression <= gateways[gateways].max_datarates[slice])
# ajustar em caso de nº gateways > 1

# --------------
# QoS Constraint
# --------------

model.qos_constraint = pyomo.ConstraintList()
qos_expressions = {}
qos_datarate_expressions = {}
qos_delay_expressions = {}
for gateway in model.M:
    for device in model.K:
        for slice in slices.keys():
            # Equivalent to S(k,l)
            if slice_associations[device] != slice:
                continue

            datarate_expression = sum(
                model.x[gateway, position, device, config] *
                (configurations[config].sf * gateways[gateway].bandwidths[slice] /
                        (2 ** configurations[config].sf)) / MAX_RKC
                for position in model.P
                for config in model.C
            )

            delay_expression = sum(
                model.x[gateway, position, device, config] *
                ( 1 - ( PACKET_SIZE /
                    (configurations[config].sf * gateways[gateway].bandwidths[slice] /
                    (2 ** configurations[config].sf))
                ) / MAX_DELAY )
                for position in model.P
                for config in model.C
            )

            expression = sum(
                model.x[gateway, position, device, config] * (
                    # r_{k,m}
                    (4/5 * configurations[config].sf * gateways[gateway].bandwidths[slice] /
                        (2 ** configurations[config].sf)) / MAX_RKC +
                    # 1 - d_{k,m}
                    ( 1 - (
                        PACKET_SIZE / (configurations[config].sf *
                        gateways[gateway].bandwidths[slice] / (2 ** configurations[config].sf))
                    ) / MAX_DELAY )
                )
                for position in model.P
                for config in model.C
            )

            model.qos_constraint.add(expression >= QOS_LOWER_BOUND)
            qos_expressions[(gateway, device)] = expression
            qos_datarate_expressions[(gateway, device)] = datarate_expression
            qos_delay_expressions[(gateway, device)] = delay_expression

# ---------------
# Loss Constraint
# ---------------
loss_constraints = {}
endDevicePositions = {}

model.loss_constraint = pyomo.ConstraintList()
for gateway in model.M:
    for position in model.P:
        gw_p = [gateway_positions[position].x, gateway_positions[position].y, gateway_positions[position].z]
        for device in model.K:
            ed_p = [end_device_positions[device].x, end_device_positions[device].y, end_device_positions[device].z]
            endDevicePositions[device] = Position(end_device_positions[device].x, end_device_positions[device].y, end_device_positions[device].z)
            distance = math.sqrt((gw_p[0] - ed_p[0])**2 + (gw_p[1] - ed_p[1])**2 + (gw_p[2] - ed_p[2])**2)/referenceDistance
            for config in model.C:
                pathLoss = 0.0
                if distance > referenceDistance:
                    pathLoss = 10 * attenuationExponent * math.log10(distance/referenceDistance)

                model.loss_constraint.add(model.x[gateway, position, device, config] * 
                                          (configurations[config].tp - referencePrx - pathLoss) 
                                          >= sensitivity[configurations[config].sf])


# model.ceil4_lower_bound = pyomo.ConstraintList()
# model.ceil4_higher_bound = pyomo.ConstraintList()

# for position in model.P:
#     for sf in spreading_factors:
#         ceil_expr = (sum(
#             model.x[gateway, position, device, config]
#             for gateway in model.M
#             for device in model.K
#             for config in model.C
#             if configurations[config].sf == sf
#         ) - 1) / len(model.K)
#         model.ceil4_lower_bound.add(model.ceil4[gateway, sf] - ceil_expr >= 0.0)
#         model.ceil4_higher_bound.add(model.ceil4[gateway, sf] - ceil_expr <= 1.0 - (1 / len(model.K)))

#     for device in model.K:
#         expression = sum(
#             model.x[gateway, position, device, config] * (
#                 # PLR'
#                     (0 if config in plr1[(device, position)] else 1) +
#                     # PLR''
#                     (1 - pyomo.exp(-2 * TRANSMISSION_PROBABILITY *
#                                    # d_{k,m}
#                                    (PACKET_SIZE / (
#                                            configurations[config].sf * gateways[gateway].bandwidths[slice] /
#                                            (2 ** configurations[config].sf)
#                                    )) *
#                                    # number of devices using the same SF
#                                    sum(
#                                        model.x[plr3_gateway, position, plr3_device, plr3_config]
#                                        for plr3_gateway in model.M
#                                        for plr3_device in model.K
#                                        for plr3_config in model.C
#                                        if configurations[plr3_config].sf == configurations[config].sf
#                                    )
#                                    )) +
#                     # PLR'''
#                     model.ceil4[gateway, configurations[config].sf]
#             )
#             for gateway in model.M
#             for config in model.C
#         )

#         # model.loss_constraint.add(expression <= LOSS_HIGHER_BOUND)
#         loss_constraints[(position, device)] = expression

# ---------------
#    Solution
# ---------------

startSolv = timer()
opt = pyomo.SolverFactory('scip')
result = opt.solve(model, options={'limits/time':1800}, tee=True)

# logging.basicConfig(level=logging.DEBUG, encoding='utf-8')
# print("------------------------ INFEASIBILITY ------------------------")
# log_infeasible_constraints(model)

startPos = timer()

gatewaysPositions = "id,x,y,z\n"
devicesConfigurations = "device,sf,tp\n"

uniqueGatewayPositions = []
deviceGatewayAssociation = {}
devicesPerSF = {7: 0, 8: 0, 9: 0, 10: 0, 11: 0, 12: 0}
for key in model.x:
    if model.x[key].value > 10**(-6):
        print('drone={}, {}, device={}, {} -> {}'.format(key[0], gateway_positions[key[1]], 
                                                         key[2], configurations[key[3]], 
                                                         model.x[key].value))
        deviceGatewayAssociation[key[2]] = key[1]
        devicesPerSF[configurations[key[3]].sf] += 1
        gatewayPosition = gateway_positions[key[1]]
        if gatewayPosition not in uniqueGatewayPositions:
            gatewaysPositions += str(str(key[1]) + "," + str(gatewayPosition.x) + "," 
                                     + str(gatewayPosition.y) + "," + str(gatewayPosition.z) + "\n")
            uniqueGatewayPositions.append(gatewayPosition)

        devicesConfigurations += str(key[2]) + "," + str(configurations[key[3]].sf) + "," + str(configurations[key[3]].tp) + "\n"

for key in model.ceil1:
    if model.ceil1[key].value != 0:
        print(str(model.ceil1[key]), ' -> ', model.ceil1[key].value)
for key in model.ceil2:
    if model.ceil1[key].value != 0:
        print(str(model.ceil2[key]), ' -> ', model.ceil2[key].value)
for key in model.ceil3:
    if model.ceil1[key].value != 0:
        print(str(model.ceil3[key]), ' -> ', model.ceil3[key].value)
for key in model.ceil4:
    if model.ceil4[key].value != 0:
        print(str(model.ceil4[key]), ' -> ', model.ceil4[key].value)

# lossResults = ""
# for gateway, expr in loss_constraints.items():
#     if pyomo.value(expr) != 0:
#         print('Loss Constraint (GatewayPosition={}, Device={}) -> '.format(gateway[0], gateway[1]), pyomo.value(expr))
#         lossResults += str(gateway[0]) + " " + str(gateway[1]) + " " + str(pyomo.value(expr)) + "\n"

qosResults = "gateway,device,qos,datarate,delay\n"
for key in qos_expressions.keys():
    qosResults += str(str(deviceGatewayAssociation[key[1]]) + "," + str(key[1]) + "," 
                      + str(pyomo.value(qos_expressions[key])) + "," 
                      + str(pyomo.value(qos_datarate_expressions[key])) + "," 
                      + str(pyomo.value(qos_delay_expressions[key])) + "\n")

devicePositions = "device,x,y,z\n"
for key in endDevicePositions.keys():
    position = endDevicePositions[key]
    devicePositions += str(str(key) + "," + str(position.x) + "," 
                           + str(position.y) + "," + str(position.z) + "\n")

solutions = str(str(args.seed) + "," + str(args.nGat) + "," + str(args.nDev) + "," + str(result.solver.time) + ","
                + str(pyomo.value(model.OBJECTIVE)) + "," + str(result.solver.gap) + ",-1.0," 
                + str(len(uniqueGatewayPositions)) + "," + str(devicesPerSF[7]) + "," + str(devicesPerSF[8]) + "," 
                + str(devicesPerSF[9]) + "," + str(devicesPerSF[10]) + "," + str(devicesPerSF[11]) + "," + str(devicesPerSF[12]) + "\n")

fileGwPlacement =  str(path_output + prefix + "_Placement_" + str(args.seed) + "s_" 
                       + str(args.nGat) + "x" + str(args.nPla) +"Gv_" + str(args.nDev) + "D.dat")                   
with open(fileGwPlacement, "w+") as outfile:
    outfile.write(gatewaysPositions)

fileDevicePlacement =  str(path_output + prefix + "_DevicePlacement_" + str(args.seed) + "s_" 
                           + str(args.nGat) + "x" + str(args.nPla) +"Gv_" + str(args.nDev) + "D.dat")                   
with open(fileDevicePlacement, "w+") as outfile:
    outfile.write(devicePositions)

fileCfgPlacement =  str(path_output + prefix + "_DevicesConfigurations_" + str(args.seed) + "s_" 
                        + str(args.nGat) + "x" + str(args.nPla) +"Gv_" + str(args.nDev) + "D.dat")
with open(fileCfgPlacement, "w+") as outfile:
    outfile.write(devicesConfigurations)

fileQoS = str(path_output + prefix + "_QoSResults_" + str(args.seed) + "s_" 
              + str(args.nGat) + "x" + str(args.nPla) +"Gv_" + str(args.nDev) + "D.dat")
with open(fileQoS, "w+") as outfile:
    outfile.write(qosResults)

fileSolutions =  str(path_output + prefix + "_solutions.dat")  
if os.path.isfile(fileSolutions):
    with open(fileSolutions, "a+") as outfile:
        outfile.write(solutions)
else:
    with open(fileSolutions, "w+") as outfile:
        outfile.write("seed,numVirtualPositions,numDevices,solveTime,objective,gap,maxConfigurations,numUAVs,sf7,sf8,sf9,sf10,sf11,sf12\n")
        outfile.write(solutions)

#DEBUG Output filenames
if DEBUG:
    print(fileGwPlacement)
    print(fileCfgPlacement)
    print(fileQoS)

endPos = timer()

print("Obj: " + str(pyomo.value(model.OBJECTIVE)))

print('Pre processing elapsed time:' + str(startSolv - startPre) + 'sec')
print('Processing elapsed time:' + str(startPos - startSolv) + 'sec')
print('Pre processing elapsed time:' + str(endPos - startPos) + 'sec')
print('Elapsed time:' + str(endPos - startPre) + 'sec')

os.system("date")
