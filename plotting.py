import csv
import matplotlib.pyplot as plt

def safe_eval(value):
    try:
        return float(value)
    except ValueError:
        return value

# Open the CSV file and read the data
with open('/Users/admin/Downloads/output.csv', 'r') as file:
    # Read each row as a single string
    rows = file.readlines()
    
    # Initialize parsed data list
    parsed_data = []
    for row in rows:
        row_data = row.strip().split(',')
        # Remove double quotes from each string in row_data
        row_data = [value.strip('"') for value in row_data]
        # Safely evaluate each alternate value
        parsed_row = dict(zip(row_data[::2], map(safe_eval, row_data[1::2])))
        parsed_data.append(parsed_row)

    # Extract x and y values from parsed data
    x_values = [line['itteration number'] for line in parsed_data if 'itteration number' in line]
    y_values_delta = [line['delta'] for line in parsed_data if 'delta' in line]
    y_values_reflected_light = [line['reflected light reading'] for line in parsed_data if 'reflected light reading' in line]

# Plot delta
plt.figure(figsize=(12, 6))

plt.subplot(2, 1, 1)
plt.plot(x_values, y_values_delta, marker='o', linestyle='-', color='b')
plt.xlabel('Iteration Number')
plt.ylabel('Delta')
plt.title('Error per Iteration')
plt.grid(True)

# Plot reflected light reading
plt.subplot(2, 1, 2)
plt.plot(x_values, y_values_reflected_light, marker='o', linestyle='-', color='r')
plt.axhline(y=50, color='g', linestyle='--', label='Setpoint')  # Add horizontal line at y=50
plt.xlabel('Iteration Number')
plt.ylabel('Reflected Light Reading')
plt.title('Reflected Light Reading per Iteration')
plt.legend()  # Show the legend for the Setpoint line
plt.grid(True)

# Add a heading for the entire figure
plt.suptitle('P=1.15, I=3.1, D=0.004, S=0', fontsize=18)

plt.tight_layout(rect=[0, 0, 1, 0.95])  # Adjust layout to make room for subtitle
plt.show()
