import influx2_client as influx



res = influx.get_measurements_list()
print(res)
