Import("env")
import os
import shutil

def copy_data(source, target, env):
    # Crear directorio data si no existe
    data_dir = os.path.join(env.get("PROJECT_DIR"), "data")
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)
    
    # Copiar archivos de data al sistema de archivos
    fs_dir = os.path.join(env.get("PROJECT_DIR"), ".pio", "build", env.get("PIOENV"), "data")
    if not os.path.exists(fs_dir):
        os.makedirs(fs_dir)
    
    # Copiar todos los archivos de data
    for file in os.listdir(data_dir):
        src = os.path.join(data_dir, file)
        dst = os.path.join(fs_dir, file)
        if os.path.isfile(src):
            shutil.copy2(src, dst)
            print(f"Copiado {file} a sistema de archivos")

env.AddPreAction("buildfs", copy_data)
env.AddPreAction("uploadfs", copy_data)