import csv

def filter_points_by_distance(input_file, output_file, threshold=0.35):
    filtered_points = []
    last_x = None

    with open(input_file, 'r') as infile:
        reader = csv.reader(infile)
        for row in reader:
            if not row or len(row) < 1:
                continue
            try:
                x = float(row[0])
            except ValueError:
                continue  # skip invalid lines

            if last_x is None or abs(x - last_x) >= threshold:
                filtered_points.append(row)
                last_x = x

    with open(output_file, 'w', newline='') as outfile:
        writer = csv.writer(outfile)
        writer.writerows(filtered_points)

# Example usage:
filter_points_by_distance('xyhead_highbay_backlot_p.csv', 'easy_p.csv')
