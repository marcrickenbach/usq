ADC_RESOLUTION = 12
ADC_MAX_VALUE = 2**ADC_RESOLUTION - 1
LUT_SIZE = ADC_MAX_VALUE + 1

# Adjust group_size to control smoothing: smaller values = finer resolution
group_size = 20  # Example value, adjust based on desired smoothing

lut = [0] * LUT_SIZE

for i in range(LUT_SIZE):
    # Map each group of input values to the midpoint of the group
    group_midpoint = ((i // group_size) * group_size) + (group_size // 2)
    lut[i] = min(group_midpoint, ADC_MAX_VALUE)  # Ensure not to exceed ADC_MAX_VALUE

# Print the LUT (or part of it, for brevity)
print(lut)  # Print the first 100 entries as an example
