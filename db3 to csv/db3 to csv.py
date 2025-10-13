import sqlite3
import csv
import os
import sys

def db3_to_csv(db3_path, output_dir=None):
    # Validate input file
    if not os.path.exists(db3_path):
        print(f"Error: file '{db3_path}' not found.")
        return

    # Default output directory
    if output_dir is None:
        output_dir = os.path.splitext(db3_path)[0] + "_csv"

    os.makedirs(output_dir, exist_ok=True)

    # Connect to the SQLite (.db3) database
    conn = sqlite3.connect(db3_path)
    cursor = conn.cursor()

    # Get all table names
    cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
    tables = cursor.fetchall()

    if not tables:
        print("No tables found in the database.")
        return

    for (table_name,) in tables:
        csv_path = os.path.join(output_dir, f"{table_name}.csv")
        print(f"Exporting table '{table_name}' to '{csv_path}'...")

        # Query all data from table
        cursor.execute(f"SELECT * FROM {table_name}")
        rows = cursor.fetchall()

        # Get column names
        column_names = [description[0] for description in cursor.description]

        # Write to CSV
        with open(csv_path, "w", newline='', encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(column_names)
            writer.writerows(rows)

    conn.close()
    print(f"\nConversion complete! CSV files saved to: {output_dir}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python db3_to_csv.py <database.db3> [output_folder]")
    else:
        db3_path = sys.argv[1]
        output_dir = sys.argv[2] if len(sys.argv) > 2 else None
        db3_to_csv(db3_path, output_dir)
