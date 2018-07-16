Import("env")
src_filter = ["+<TRB_INA219.h>"]
env.Replace(SRC_FILTER=src_filter)

build_flags = env.ParseFlags(env['BUILD_FLAGS'])
cppdefines = build_flags.get("CPPDEFINES")

if "TRB_INA219_I2C_WIRE" in cppdefines:
    env.Append(SRC_FILTER=["+<TRB_INA219.cpp>"])
if "TRB_INA219_I2C_ESP_IDF" in cppdefines:
    env.Append(SRC_FILTER=["+<TRB_INA219.c>"])
if "TRB_INA219_I2C_BRZO" in cppdefines:
    env.Append(SRC_FILTER=["+<TRB_INA219.cpp>"])
print env.get("SRC_FILTER")
