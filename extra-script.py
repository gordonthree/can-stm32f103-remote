Import("env")

f_index = env.get("UPLOADERFLAGS", []).index("-f")
hla_serial = env.GetProjectOption("custom_hla_serial", None)
if f_index > 0 and hla_serial:
    env["UPLOADERFLAGS"].insert(f_index, ["-c", f"adapter serial {hla_serial}"])
