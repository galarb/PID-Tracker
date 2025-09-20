import matplotlib.pyplot as plt

def read_pid_file(filename):
    t_list, e_list, d_list = [], [], []
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.lower().startswith("time"):
                continue
            parts = line.split(",")
            t_list.append(float(parts[0]))
            e_list.append(float(parts[1]))   # error
            d_list.append(float(parts[2]))   # derivative (adjust index if needed)
    return t_list, e_list, d_list

# Filenames for the 4 runs
files = [
    ("pidvalues.txt", "1.8,0,0, alpha=0.2"),
    ("pidvalues2.txt", "1.8,0,0.2"),
    ("pidvalues3.txt", "1.8,0,0.4"),
    ("pidvalues4.txt", "1.8,0,0.6, alpha=2"),
]

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10,10), sharex=True)

for filename, label in files:
    t, e, d = read_pid_file(filename)
    ax1.plot(t, e, label=label)
    ax2.plot(t, d, label=label)

ax1.set_ylabel("Error")
ax1.set_title("PID Error Comparison (4 runs)")
ax1.legend()
ax1.grid(True)

ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Derivative")
ax2.set_title("PID Derivative Comparison (4 runs)")
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.show()

