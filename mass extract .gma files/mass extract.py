import os
import subprocess
import re
import shutil
from pathlib import Path
from tkinter import Tk, filedialog, simpledialog, messagebox

def select_folder(title):
    """Open a folder selection dialog."""
    root = Tk()
    root.withdraw()
    root.attributes('-topmost', True)
    folder = filedialog.askdirectory(title=title)
    root.destroy()
    return folder

def get_password():
    """Prompt user for password with a hint."""
    root = Tk()
    root.withdraw()
    root.attributes('-topmost', True)
    password = simpledialog.askstring(
        "Password Required for extract_archive.bat",
        "Enter the decryption password:\n(Hint: The password is 'M3thu3n')",
        show='*'
    )
    root.destroy()
    return password

def find_most_recent_case_json(extracted_folder):
    """Find the most recent _case.json file in workflow_case/state folder."""
    state_path = Path(extracted_folder) / "workflow_case" / "state"
    
    if not state_path.exists():
        return None
    
    # Find all files matching the pattern state_XXXXXX_*_case.json
    case_files = list(state_path.glob("state_*_case.json"))
    
    if not case_files:
        return None
    
    # Extract the 6-digit number after "state_" and find the highest
    max_num = -1
    most_recent_file = None
    
    for file in case_files:
        match = re.match(r'state_(\d{6})_.*_case\.json', file.name)
        if match:
            num = int(match.group(1))
            if num > max_num:
                max_num = num
                most_recent_file = file
    
    return most_recent_file

def extract_gma_file(gma_file, extract_bat, progress_msg):
    """Extract a single .gma file using the batch script."""
    print(progress_msg)
    
    try:
        process = subprocess.Popen(
            [str(extract_bat), str(gma_file)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            cwd=gma_file.parent
        )
        
        stdout, stderr = process.communicate()
        
        if process.returncode != 0:
            return False, f"Extraction failed with return code {process.returncode}\n{stderr}"
        
        return True, None
        
    except Exception as e:
        return False, str(e)

def main():
    print("=== Mass GMA File Extractor ===\n")
    
    # Check if GM_HOME environment variable exists
    if not os.getenv('GM_HOME'):
        messagebox.showerror("Error", "GM_HOME environment variable is not set! This script is intended to be run on a dev computer")
        return
    
    # Step 1: Select folder containing .gma files
    source_folder = select_folder("Select folder containing .gma files")
    if not source_folder:
        print("No folder selected. Exiting.")
        return
    
    source_path = Path(source_folder)

    # Check if extract_archive_no_password.bat exists in the same folder as this script
    script_dir = Path(__file__).parent
    extract_bat = script_dir / "extract_archive_no_password.bat"
    if not extract_bat.exists():
        messagebox.showerror("Error", f"extract_archive_no_password.bat not found in {extract_bat}")
        return
    
    # Find all .gma files
    gma_files = list(source_path.glob("*.gma"))
    if not gma_files:
        messagebox.showinfo("No Files", "No .gma files found in the selected folder.")
        return
    
    print(f"Found {len(gma_files)} .gma file(s)\n")
    
    # Step 6: Select output directory for JSON files
    output_folder = select_folder("Select folder to save extracted JSON files")
    if not output_folder:
        print("No output folder selected. Exiting.")
        return
    
    output_path = Path(output_folder)
    
    # Track results
    successful = []
    failed_extraction = []
    no_json_found = []
    
    # Process each .gma file
    for idx, gma_file in enumerate(gma_files, 1):
        progress_msg = f"[{idx}/{len(gma_files)}] Processing: {gma_file.name}"
        
        # Expected extracted folder name (same as .gma filename without extension)
        extracted_folder = source_path / gma_file.stem
        
        # Step 2-4: Extract the file
        success, error = extract_gma_file(gma_file, extract_bat, progress_msg)
        
        if not success:
            failed_extraction.append((gma_file.name, error))
            continue
        
        # Step 5: Find the most recent _case.json file
        case_json = find_most_recent_case_json(extracted_folder)
        
        if not case_json:
            no_json_found.append(gma_file.name)
            # Step 7: Clean up extracted folder
            if extracted_folder.exists():
                shutil.rmtree(extracted_folder)
            continue
        
        # Step 6: Copy JSON file with new name
        # Extract state number from filename
        match = re.match(r'state_(\d{6})_', case_json.name)
        if match:
            state_num = match.group(1)
            new_filename = f"{gma_file.stem}_state_{state_num}.json"
            destination = output_path / new_filename
            
            try:
                shutil.copy2(case_json, destination)
                successful.append((gma_file.name, new_filename))
            except Exception as e:
                failed_extraction.append((gma_file.name, f"Failed to copy JSON: {str(e)}"))
        
        # Step 7: Delete extracted folder
        try:
            if extracted_folder.exists():
                shutil.rmtree(extracted_folder)
        except Exception as e:
            print(f"  ⚠ Warning: Could not delete {extracted_folder}: {str(e)}")
        
        print()
    
    # Step 8: Display summary
    print("\n" + "="*60)
    print("PROCESSING SUMMARY")
    print("="*60)
    
    print(f"\n✓ Successfully Processed ({len(successful)}):")
    for gma_name, json_name in successful:
        print(f"  - {gma_name} → {json_name}")
    
    if failed_extraction:
        print(f"\n✗ Failed to Extract ({len(failed_extraction)}):")
        for gma_name, error in failed_extraction:
            print(f"  - {gma_name}: {error}")
    
    if no_json_found:
        print(f"\n⚠ No JSON Found ({len(no_json_found)}):")
        for gma_name in no_json_found:
            print(f"  - {gma_name}")
    
    print(f"\nTotal files processed: {len(gma_files)}")
    print("="*60)
    
    # Show summary dialog
    summary = f"Processing Complete!\n\n"
    summary += f"Successfully processed: {len(successful)}\n"
    summary += f"Failed extractions: {len(failed_extraction)}\n"
    summary += f"No JSON found: {len(no_json_found)}\n"
    summary += f"\nJSON files saved to:\n{output_folder}"
    
    messagebox.showinfo("Summary", summary)

if __name__ == "__main__":
    main()