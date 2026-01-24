# name as "subsystem_name"
name = input("Name of the subsystem in snake_case: ")

# name as "SubsystemName"
name_pascal = ''.join(word.title() for word in name.split('_'))
# name as "subsystemName"
name_camel = name_pascal[0].lower() + name_pascal[1:]
# name as "SUBSYSTEM_NAME"
name_upper = name.upper()

substitutions = {
    '$Name': name_pascal,
    '$name': name_camel,
    '$NAME': name_upper
}

# Copy all files from template/ to src/main/java/frc/robot/subsystems/{name}/
# and replace $name in the filename and content with the provided name.
import os
import shutil

template_dir = os.path.join(os.path.dirname(__file__), 'template')
target_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'src', 'main', 'java', 'frc', 'robot', 'subsystems', name_camel)
os.makedirs(target_dir, exist_ok=True)

for root, dirs, files in os.walk(template_dir):
    for file in files:
        rel_dir = os.path.relpath(root, template_dir)
        rel_file = os.path.join(rel_dir, file)
        
        # Replace $name in filename
        target_file_name = file
        for key, value in substitutions.items():
            target_file_name = target_file_name.replace(key, value)
        
        target_file_path = os.path.join(target_dir, rel_dir, target_file_name)
        os.makedirs(os.path.dirname(target_file_path), exist_ok=True)
        
        # Read template file content
        with open(os.path.join(root, file), 'r') as f:
            content = f.read()
        
        # Replace $name in content
        for key, value in substitutions.items():
            content = content.replace(key, value)
        
        # Write to target file
        with open(target_file_path, 'w') as f:
            f.write(content)

print(f"Subsystem '{name}' created successfully in {target_dir}.")