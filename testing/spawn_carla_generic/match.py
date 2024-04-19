import re

# Example text that might come from a file or directly as a string
sample_text = """
Create 167: vehicle.tesla.cybertruck (1) at (-2433.68, -5778.56, 60)
Id: 167 Location: (-2433.68, -5778.56, 60)
Create 168: sensor.other.collision (5) at (0, 0, 0)
Id: 168 Location: (0, 0, 0)
Create 169: sensor.other.lane_invasion (5) at (0, 0, 0)
Id: 169 Location: (0, 0, 0)
"""

def do_match(text):
    # Regex pattern to find lines with 'Create xxx: vehicle.yyy'
    pattern = r"(Create \d+: vehicle\.\S+.*$)"

    # Use re.findall to find all occurrences that match the pattern
    matches = re.findall(pattern, text, re.MULTILINE)

    # Print out the results
    results = []
    for line in matches:
        # Extract the vehicle ID and type from the matched line
        id_and_type = re.search(r"Create (\d+): vehicle\.(\S+)", line)
        if id_and_type:
            vehicle_id, vehicle_type = id_and_type.groups()
            print(f"Line: {line.strip()}")
            print(f"Vehicle ID: {vehicle_id}, Vehicle Type: {vehicle_type}\n")
            results.append((vehicle_id, line))
    return results


def do_match_full(text, vehicle_id):
    # Split the text into lines
    lines = text.splitlines()

    # Dynamic pattern based on provided vehicle_id
    pattern = rf"Id: {vehicle_id} Location.*"

    # List to hold the results
    results = []

    # Iterate over lines with their indices for easy reference to previous lines
    for index, line in enumerate(lines):
        if re.search(pattern, line):
            # Check if there's a line three places before and add it
            if index >= 3:
                previous_line = lines[index - 3]
                if previous_line.startswith("Frame"):
                    if (results and results[-1] != line) or results == []:
                        # Match found, prepare to append this line and the line three before, if possible
                        print(f"Matching Line: {line.strip()}")
                        print(f"Line 3 places before: {previous_line.strip()}")

                        results.append(previous_line)
                        results.append(line)
    return results


if __name__ == "__main__":
    vehicle_matches = do_match(sample_text)
    if len(vehicle_matches) > 0 and len(vehicle_matches[0]) > 0:
        # Assuming we want to further search based on the first vehicle ID found
        first_vehicle_id = vehicle_matches[0][0]
        do_match_full(sample_text, first_vehicle_id)
