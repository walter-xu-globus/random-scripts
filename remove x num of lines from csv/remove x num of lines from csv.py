def remove_lines_from_csv(input_file, output_file, lines_to_remove=0):
    try:
        with open(input_file, 'r') as infile:
            with open(output_file, 'w') as outfile:
                for _ in range(lines_to_remove):
                    infile.readline()
                
                for line in infile:
                    outfile.write(line)
        
        print(f"Successfully removed the first {lines_to_remove} lines and saved the result to '{output_file}'.")
    
    except FileNotFoundError:
        print(f"Error: The file '{input_file}' does not exist.")
    except Exception as e:
        print(f"An error occurred: {e}")

input_csv_file = 'GlobusGMED5 not working from 10.54am.csv'
output_csv_file = 'GlobusGMED5 not working from 10.54am trimmed.csv'

remove_lines_from_csv(input_csv_file, output_csv_file, lines_to_remove=200000)