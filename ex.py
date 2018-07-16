Import("env")
src_filter = ["+<INA219C.h>"]
env.Replace(SRC_FILTER=src_filter)

build_flags = env.ParseFlags(env['BUILD_FLAGS'])
cppdefines = build_flags.get("CPPDEFINES")

if "INA219C_I2C_WIRE" in cppdefines:
    env.Append(SRC_FILTER=["+<INA219C.cpp>"])
if "INA219C_I2C_ESP_IDF" in cppdefines:
    env.Append(SRC_FILTER=["+<INA219C.c>"])
if "INA219C_I2C_BRZO" in cppdefines:
    env.Append(SRC_FILTER=["+<INA219C.cpp>"])
print env.get("SRC_FILTER")
