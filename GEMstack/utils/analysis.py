import os
import pandas as pd
import numpy as np

def list_log_directories():
    logs_path = "./logs"
    if not os.path.exists(logs_path):
        print("Logs directory does not exist.")
        return []
    
    directories = [d for d in os.listdir(logs_path) if os.path.isdir(os.path.join(logs_path, d))]
    return directories

def choose_directory(directories):
    if not directories:
        print("No log directories found.")
        return None
    
    print("Available log directories:")
    for i, dir_name in enumerate(directories):
        print(f"{i + 1}. {dir_name}")
    
    choice = int(input("Select a directory by number: ")) - 1
    if 0 <= choice < len(directories):
        return os.path.join("./logs", directories[choice])
    else:
        print("Invalid choice.")
        return None

def choose_csv_file(log_dir):
    files = [f for f in os.listdir(log_dir) if f.endswith(".csv")]
    if not files:
        print("No CSV files found in the selected directory.")
        return None
    
    print("Available CSV files:")
    for i, file in enumerate(files):
        print(f"{i + 1}. {file}")
    
    choice = int(input("Select a CSV file by number: ")) - 1
    if 0 <= choice < len(files):
        return os.path.join(log_dir, files[choice])
    else:
        print("Invalid choice.")
        return None

def load_csv(csv_path):
    if not os.path.exists(csv_path):
        print("CSV file not found.")
        return None
    
    df = pd.read_csv(csv_path)
    print("CSV file loaded successfully.")
    print("Available columns:", list(df.columns))
    return df

def choose_columns(df):
    selected_columns = input("Enter column names to analyze (comma separated): ").split(',')
    selected_columns = [col.strip() for col in selected_columns if col.strip() in df.columns]
    if not selected_columns:
        print("No valid columns selected.")
        return None
    return df[selected_columns]

def analyze_data(df, log_dir):
    methods = {
        "1": ("Mean Squared Error (MSE)", lambda x: np.mean(np.square(x))),
        "2": ("Root Mean Square (RMS)", lambda x: np.sqrt(np.mean(np.square(x)))),
        "3": ("Mean", np.mean),
        "4": ("Standard Deviation", np.std),
        "5": ("Custom Lambda Function", None)
    }
    
    print("Available analysis methods:")
    for key, (name, _) in methods.items():
        print(f"{key}. {name}")
    
    choice = input("Select a method by number: ")
    if choice in methods:
        if choice == "5":
            function_str = input("Enter a lambda function (e.g., lambda x: np.max(x) - np.min(x)): ").strip()
            try:
                func = eval(function_str, {"np": np})
                result = df.apply(func)
            except Exception as e:
                print(f"Invalid function: {e}")
                return
        else:
            method_name, method_func = methods[choice]
            print(f"Applying {method_name}...")
            result = df.apply(method_func)
        
        print("Analysis result:")
        print(result.to_string(header=True, index=True))
        
        save_path = input("Enter a path to save the result (press Enter to save in the chosen log directory): ")
        if not save_path:
            save_path = os.path.join(log_dir, "analysis_result.csv")
        
        result.to_frame().T.to_csv(save_path, index=False, header=True)
        print(f"Analysis result saved to {save_path}")
    else:
        print("Invalid choice.")

def main():
    directories = list_log_directories()
    log_dir = choose_directory(directories)
    if not log_dir:
        return
    
    csv_path = choose_csv_file(log_dir)
    if not csv_path:
        return
    
    df = load_csv(csv_path)
    if df is None:
        return
    
    df_selected = choose_columns(df)
    if df_selected is None:
        return
    
    analyze_data(df_selected, log_dir)

if __name__ == "__main__":
    main()
