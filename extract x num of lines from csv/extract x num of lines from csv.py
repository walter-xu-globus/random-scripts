def extract_lines_from_csv(input_file, output_file, start=0, end=-1):
    try:
        with open(input_file, 'r') as infile:
            lines = infile.readlines()

        # If end is -1, read till the end
        if end == -1:
            end = len(lines)

        # Slice the desired range
        extracted_lines = lines[start:end]

        with open(output_file, 'w') as outfile:
            outfile.writelines(extracted_lines)

        print(f"Successfully extracted lines {start} to {end - 1} (inclusive) and saved to '{output_file}'.")

    except FileNotFoundError:
        print(f"Error: The file '{input_file}' does not exist.")
    except Exception as e:
        print(f"An error occurred: {e}")

input_csv_file = 'driveExportedLogs14-May-2025_19-50-43.csv'
output_csv_file = 'driveExportedLogs14-May-2025_19-50-43 trimmed.csv'

# zero indexed line num
extract_lines_from_csv(input_csv_file, output_csv_file, start=504042, end=-1)
