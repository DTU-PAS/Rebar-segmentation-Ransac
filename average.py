import re
import sys

def calculate_averages(file_path):
    # Initialize lists to store the distances
    vertical_distances = []
    horizontal_distances = []

    # Read the file
    with open(file_path, 'r') as file:
        for line in file:
            # Search for vertical distances
            vertical_match = re.search(r'Vertical Distance:\s*([\d.]+)\s*mm', line)
            if vertical_match:
                vertical_distances.append(float(vertical_match.group(1)))

            # Placeholder for future horizontal distances
            horizontal_match = re.search(r'Horizontal Distance:\s*([\d.]+)\s*mm', line)
            if horizontal_match:
                horizontal_distances.append(float(horizontal_match.group(1)))

    # Calculate the averages
    avg_vertical = sum(vertical_distances) / len(vertical_distances) if vertical_distances else 0
    avg_horizontal = sum(horizontal_distances) / len(horizontal_distances) if horizontal_distances else 0

    return avg_vertical, avg_horizontal

# Path to the uploaded file
args = sys.argv
file_path = args[1]

# Calculate and print the averages
avg_vertical, avg_horizontal = calculate_averages(file_path)
print(f"Average Vertical Distance: {avg_vertical:.4f} mm")

if avg_horizontal > 0:
    print(f"Average Horizontal Distance: {avg_horizontal:.4f} mm")
else:
    print("No horizontal distances found in the file.")
