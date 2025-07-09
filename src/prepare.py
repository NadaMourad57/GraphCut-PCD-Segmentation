import os 
import shutil

base_folder=os.path.join(os.getcwd(),"exported_blades_v3")
data_folder=os.path.join(base_folder,"Dataset")

if not os.path.exists(data_folder):
    os.mkdir(os.path.join(base_folder,"Dataset"))

#loop through all the folders in the base folder
for folder in os.listdir(base_folder):
    #check if the folder is a directory
    if os.path.isdir(os.path.join(base_folder, folder)) and folder != "Dataset":
        # Create corresponding folder in Dataset if it doesn't exist
        target_folder = os.path.join(data_folder, folder)
        if not os.path.exists(target_folder):
            os.mkdir(target_folder)
        
        #loop through all the files in the folder
        for file in os.listdir(os.path.join(base_folder, folder)):
            #check if the file is a .ply file
            if (file.endswith(".ply") or file.endswith(".obj")) and not file.startswith("."):
                file_path = os.path.join(base_folder, folder, file)
                # Copy the .ply file to the corresponding folder in Dataset
                target_path = os.path.join(target_folder, file)
                shutil.copy2(file_path, target_path)
                print(f"Copied {file_path} to {target_path}")
